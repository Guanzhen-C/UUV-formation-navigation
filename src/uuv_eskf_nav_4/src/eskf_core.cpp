#include "uuv_eskf_nav/eskf_core.h"
#include <iostream>
#include <cmath>

namespace uuv_eskf_nav {

EskfCore::EskfCore(const NoiseParams& noise_params, bool accel_includes_gravity)
    : noise_params_(noise_params)
    , error_state_(Eigen::VectorXd::Zero(STATE_SIZE))
    , P_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 1e-6)
    , initialized_(false)
    , has_previous_imu_(false)
    , accel_includes_gravity_(accel_includes_gravity)
    , enable_earth_rotation_(true)  // 默认关闭地球自转补偿
    , mission_latitude_(0.3183) {    // 默认纬度18.25°N (三亚)
}

bool EskfCore::initialize(const NominalState& initial_state,
                                 const Eigen::MatrixXd& initial_covariance) {
    if (initial_covariance.rows() != STATE_SIZE || initial_covariance.cols() != STATE_SIZE) {
        std::cerr << "改进ESKF初始化失败: 初始协方差矩阵维度不正确!" << std::endl;
        return false;
    }
    
    nominal_state_ = initial_state;
    error_state_.setZero();
    P_ = initial_covariance;
    has_previous_imu_ = false;
    
    initialized_ = true;
    
    std::cout << "改进ESKF已成功初始化!" << std::endl;
    std::cout << "初始位置: [" << nominal_state_.position.transpose() << "]" << std::endl;
    std::cout << "初始速度: [" << nominal_state_.velocity.transpose() << "]" << std::endl;
    std::cout << "启用功能: 圆锥误差补偿 + 划桨效应补偿" << std::endl;
    
    return true;
}
double EskfCore::computeRealtimeLatitudeRad(const Eigen::Vector3d& enu_position) const {
    // 基于 ENU 北向位移近似：φ(t) = φ0 + y/(R_M + h)，其中 φ0=mission_latitude_
    // 这里使用任务区域纬度对应的子午圈半径
    Eigen::Vector2d rmrn0 = computeEarthRadii(mission_latitude_);
    double rmh0 = rmrn0[0] + enu_position[2];
    double denom = (rmh0 > 1.0 ? rmh0 : 1.0); // 避免除零
    double dphi = enu_position[1] / denom;
    double phi = mission_latitude_ + dphi;
    // 限幅，避免 tan(phi) 数值发散
    const double lim = 1.4844222297453323; // ~85°
    if (phi > lim) phi = lim;
    if (phi < -lim) phi = -lim;
    return phi;
}

ImuIntegralData ImuIntegralData::fromInstantaneous(
    const ImuData& imu_current,
    const ImuData& imu_previous,
    double dt) {
    
    ImuIntegralData result;
    result.dt = dt;
    result.timestamp = imu_current.timestamp;
    
    // 梯形积分法转换瞬时数据为增量数据
    // 角增量：积分角速度
    result.delta_theta = 0.5 * dt * (imu_current.angular_velocity + imu_previous.angular_velocity);
    
    // 速度增量：积分线加速度
    result.delta_velocity = 0.5 * dt * (imu_current.linear_acceleration + imu_previous.linear_acceleration);
    
    return result;
}

bool EskfCore::predictWithImprovedMechanization(
    const ImuData& imu_current,
    const ImuData& imu_previous) {
    
    if (!initialized_) {
        std::cerr << "改进ESKF预测失败: 滤波器未初始化!" << std::endl;
        return false;
    }
    
    // 计算时间间隔
    double dt = imu_current.timestamp - imu_previous.timestamp;
    if (dt <= 1e-6 || dt > 0.1) {
        std::cerr << "改进ESKF预测警告: 时间间隔异常 dt=" << dt << "s" << std::endl;
        return false;
    }
    
    // 修复：先补偿偏差再积分（正确的做法）
    ImuData corrected_current = imu_current;
    ImuData corrected_previous = imu_previous;
    corrected_current.angular_velocity -= nominal_state_.gyro_bias;
    corrected_current.linear_acceleration -= nominal_state_.accel_bias;
    corrected_previous.angular_velocity -= nominal_state_.gyro_bias;
    corrected_previous.linear_acceleration -= nominal_state_.accel_bias;
    
    // 圆周运动检测
    double omega_norm = corrected_current.angular_velocity.norm();
    
    // 转换为积分数据格式（使用补偿后的数据）
    ImuIntegralData imu_curr_integral = ImuIntegralData::fromInstantaneous(
        corrected_current, corrected_previous, dt);
    
    // 保存前一时刻状态用于F和Q计算
    NominalState previous_state = nominal_state_;
    
    // ========= 按照KF-GINS标准顺序执行机械编排 =========
    // 注意：必须按照 速度→位置→姿态 的顺序更新，不可调换！
    
    // 1. 速度更新 (参考KF-GINS velUpdate)
    Eigen::Vector3d new_velocity;
    velocityUpdate(previous_state, new_velocity, imu_curr_integral, dt);
    
    // 2. 位置更新 (参考KF-GINS posUpdate) 
    Eigen::Vector3d new_position;
    positionUpdate(previous_state, new_position, new_velocity, dt);
    
    // 3. 姿态更新 (参考KF-GINS attUpdate)
    Eigen::Quaterniond new_orientation;
    attitudeUpdate(previous_state, new_orientation, new_velocity, new_position, imu_curr_integral, dt);
    
    // 4. 偏差保持不变（随机游走模型）
    Eigen::Vector3d new_gyro_bias = nominal_state_.gyro_bias;
    Eigen::Vector3d new_accel_bias = nominal_state_.accel_bias;
    
    // 5. 协方差预测 (基于前一时刻状态)
    Eigen::MatrixXd F = buildStateTransitionMatrix(dt, imu_current, previous_state);
    Eigen::MatrixXd Q = buildProcessNoiseMatrix(dt, previous_state);
    
    // 协方差预测: P = F * P * F^T + Q
    // 添加数值稳定性保护
    P_ = F * P_ * F.transpose() + Q;
    
    // 强制对称性和正定性
    P_ = 0.5 * (P_ + P_.transpose());
    
    // 协方差预测完成
    
    // 6. 更新状态 (在协方差预测之后)
    nominal_state_.position = new_position;
    nominal_state_.velocity = new_velocity;
    nominal_state_.orientation = new_orientation;
    nominal_state_.gyro_bias = new_gyro_bias;
    nominal_state_.accel_bias = new_accel_bias;
    nominal_state_.timestamp = imu_current.timestamp;
    
    // 7. 保存当前积分数据用于下次圆锥补偿
    previous_imu_integral_ = imu_curr_integral;
    has_previous_imu_ = true;
    
    return true;
}

void EskfCore::velocityUpdate(
    const NominalState& state_prev,
    Eigen::Vector3d& velocity_curr,
    const ImuIntegralData& imu_integral,
    double dt) {
    
    // 计算旋转和划桨效应补偿（参考KF-GINS第56-59行）
    Eigen::Vector3d temp1, temp2, temp3;
    if (has_previous_imu_) {
        temp1 = imu_integral.delta_theta.cross(imu_integral.delta_velocity) / 2.0;
        temp2 = previous_imu_integral_.delta_theta.cross(imu_integral.delta_velocity) / 12.0;
        temp3 = previous_imu_integral_.delta_velocity.cross(imu_integral.delta_theta) / 12.0;
    } else {
        temp1 = imu_integral.delta_theta.cross(imu_integral.delta_velocity) / 2.0;
        temp2.setZero();
        temp3.setZero();
    }
    
    // b系比力积分项
    Eigen::Vector3d d_vfb = imu_integral.delta_velocity + temp1 + temp2 + temp3;
    
    // 重要修复：精确计算中间时刻的姿态（参考KF-GINS）
    // 使用前一时刻姿态和当前角增量的一半来近似中间时刻姿态
    Eigen::Vector3d mid_theta = imu_integral.delta_theta * 0.5;
    Eigen::Quaterniond q_mid = rotationVectorToQuaternion(mid_theta);
    Eigen::Matrix3d R_bn_mid = (state_prev.orientation * q_mid).toRotationMatrix();
    
    // 比力积分项投影到n系（使用中间时刻姿态）
    Eigen::Vector3d d_vfn = R_bn_mid * d_vfb;
    
    // 重力/哥氏积分项（参考KF-GINS第71-74行）
    Eigen::Vector3d gravity_coriolis_term;
    if (accel_includes_gravity_) {
        gravity_coriolis_term.setZero();
    } else {
        // 重力项
        Eigen::Vector3d gl(0, 0, GRAVITY_ENU.z());  // 重力只有垂直分量
        
        // 哥氏力项（如果启用地球自转补偿）
        Eigen::Vector3d coriolis_term = Eigen::Vector3d::Zero();
        if (enable_earth_rotation_) {
            // 使用实时纬度(基于ENU北向位移)计算地球自转角速度和导航系转动角速度（均在ENU下）
            double current_latitude = computeRealtimeLatitudeRad(state_prev.position);
            Eigen::Vector3d wie_enu = computeEarthRotationRate(current_latitude);  // ENU
            Eigen::Vector3d wen_enu = computeNavigationFrameRate(state_prev.velocity, state_prev.position); // ENU
            // 哥氏力: -2*(w_ie + w_en) × v  （全部在 ENU 坐标系）
            coriolis_term = -(2 * wie_enu + wen_enu).cross(state_prev.velocity);
        }
        
        gravity_coriolis_term = (gl + coriolis_term) * dt;
    }
    
    // 速度更新完成
    velocity_curr = state_prev.velocity + d_vfn + gravity_coriolis_term;
}

void EskfCore::positionUpdate(
    const NominalState& state_prev,
    Eigen::Vector3d& position_curr,
    const Eigen::Vector3d& velocity_curr,
    double dt) {
    
    // 使用梯形积分法（参考KF-GINS第120-121行）
    Eigen::Vector3d midvel = (velocity_curr + state_prev.velocity) / 2.0;
    position_curr = state_prev.position + midvel * dt;
}

void EskfCore::attitudeUpdate(
    const NominalState& state_prev,
    Eigen::Quaterniond& orientation_curr,
    const Eigen::Vector3d& velocity_curr,
    const Eigen::Vector3d& position_curr,
    const ImuIntegralData& imu_integral,
    double dt) {
    
    // 计算b系旋转四元数，补偿二阶圆锥误差（参考KF-GINS第178-182行）
    Eigen::Vector3d temp1;
    if (has_previous_imu_) {
        // 完整的圆锥补偿
        temp1 = imu_integral.delta_theta + 
                previous_imu_integral_.delta_theta.cross(imu_integral.delta_theta) / 12.0;
    } else {
        temp1 = imu_integral.delta_theta;
    }
    
    // 姿态更新计算
    Eigen::Quaterniond qbb = rotationVectorToQuaternion(temp1);
    //Eigen::Quaterniond qnn = navigationFrameRotation(velocity_curr, position_curr, dt);
    
    // 姿态更新（参考KF-GINS第186行），包含导航系转动 qnn
    orientation_curr = (state_prev.orientation * qbb).normalized();
    
    // 检查四元数有效性
    if (!orientation_curr.coeffs().allFinite() || std::abs(orientation_curr.norm() - 1.0) > 0.01) {
        std::cerr << "姿态更新异常，保持原姿态" << std::endl;
        orientation_curr = state_prev.orientation;
    }
}

// 圆锥补偿已集成到attitudeUpdate中，这个函数不再需要
// Coning correction is now integrated into attitudeUpdate

Eigen::Vector3d EskfCore::scullingCorrection(
    const Eigen::Vector3d& delta_theta_curr,
    const Eigen::Vector3d& delta_vel_curr) {
    
    // 修复：实现完整的KF-GINS划桨效应补偿（三项补偿）
    // temp1 = imucur.dtheta.cross(imucur.dvel) / 2;
    // temp2 = imupre.dtheta.cross(imucur.dvel) / 12;
    // temp3 = imupre.dvel.cross(imucur.dtheta) / 12;
    
    Eigen::Vector3d sculling_correction_1 = delta_theta_curr.cross(delta_vel_curr) / 2.0;
    
    Eigen::Vector3d sculling_correction = sculling_correction_1;
    
    // 如果有前一时刻数据，添加交叉项补偿
    if (has_previous_imu_) {
        Eigen::Vector3d delta_theta_prev = previous_imu_integral_.delta_theta;
        Eigen::Vector3d delta_vel_prev = previous_imu_integral_.delta_velocity;
        
        Eigen::Vector3d sculling_correction_2 = delta_theta_prev.cross(delta_vel_curr) / 12.0;
        Eigen::Vector3d sculling_correction_3 = delta_vel_prev.cross(delta_theta_curr) / 12.0;
        
        sculling_correction += sculling_correction_2 + sculling_correction_3;
    }
    
    return sculling_correction;
}

Eigen::Quaterniond EskfCore::navigationFrameRotation(
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& position,
    double dt) {
    
    // 如果未启用地球自转补偿，返回单位四元数
    if (!enable_earth_rotation_) {
        return Eigen::Quaterniond::Identity();
    }
    
    // 计算导航系转动角速度 (ENU)，使用基于北向位移的实时纬度
    double current_latitude = computeRealtimeLatitudeRad(position);
    Eigen::Vector3d wie_enu = computeEarthRotationRate(current_latitude);
    Eigen::Vector3d wen_enu = computeNavigationFrameRate(velocity, position);
    
    // 总的导航系转动角速度
    Eigen::Vector3d omega_nn = wie_enu + wen_enu;
    
    // 转动角增量
    Eigen::Vector3d delta_theta_nn = omega_nn * dt;
    
    // 使用中间时刻角速度进行补偿 (参考KF-GINS第67行)
    Eigen::Vector3d temp1 = delta_theta_nn / 2;
    Eigen::Matrix3d cnn = Eigen::Matrix3d::Identity() - skewSymmetric(temp1);
    
    // 将旋转矩阵转换为四元数
    Eigen::Quaterniond qnn(cnn);
    qnn.normalize();
    
    return qnn;
}

Eigen::Quaterniond EskfCore::rotationVectorToQuaternion(
    const Eigen::Vector3d& rotation_vector) {
    
    double angle = rotation_vector.norm();
    
    if (angle > 1e-8) {
        // 使用Rodrigues公式的精确解
        Eigen::Vector3d axis = rotation_vector / angle;
        double half_angle = 0.5 * angle;
        double sin_half = std::sin(half_angle);
        double cos_half = std::cos(half_angle);
        
        return Eigen::Quaterniond(cos_half, 
                                 sin_half * axis.x(),
                                 sin_half * axis.y(), 
                                 sin_half * axis.z());
    } else {
        // 小角度近似
        return Eigen::Quaterniond(1.0, 
                                 0.5 * rotation_vector.x(),
                                 0.5 * rotation_vector.y(),
                                 0.5 * rotation_vector.z()).normalized();
    }
}

bool EskfCore::updateWithDvl(const DvlData& dvl_data) {
    if (!initialized_) {
        std::cerr << "改进ESKF DVL更新失败: 滤波器未初始化!" << std::endl;
        return false;
    }
    
    // 1. 构建观测模型（在body坐标系中进行对比）
    Eigen::Matrix3d R_bn = nominal_state_.orientation.toRotationMatrix();
    Eigen::Matrix3d R_nb = R_bn.transpose();
    
    // 2. 计算观测残差 (新息)
    Eigen::Vector3d predicted_body_velocity = R_nb * nominal_state_.velocity;
    Eigen::Vector3d innovation = dvl_data.velocity - predicted_body_velocity;
    
    
    // 3. 动态构建观测雅可比矩阵 H
    Eigen::MatrixXd H = buildDvlObservationMatrix(); // 3x15矩阵，内部基于当前名义状态构建
    
    // 4. 观测噪声协方差（固定，不随旋转率缩放）
    Eigen::Matrix3d R_base = Eigen::Matrix3d::Identity() * 
        (noise_params_.dvl_noise_std * noise_params_.dvl_noise_std);

    // 5. 计算卡尔曼增益
    // 基本形式：K = P * H^T * (H * P * H^T + R)^(-1)
    Eigen::MatrixXd PHt = P_ * H.transpose();
    Eigen::Matrix3d R_eff = R_base;
    Eigen::Vector3d innovation_used = innovation;
    
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_eff; // 新息协方差
    Eigen::LLT<Eigen::MatrixXd> llt_solver(S);
    if (llt_solver.info() != Eigen::Success) {
        std::cerr << "DVL新息协方差矩阵不可逆!" << std::endl;
        return false;
    }
    // 正常更新
    
    // 正常更新
    Eigen::MatrixXd K = llt_solver.solve(PHt.transpose()).transpose();
    
    // 取消增益缩放（不随旋转率调整）
    
    Eigen::VectorXd delta_error = K * innovation;
    error_state_ = delta_error;
    
    // 协方差更新 (Joseph形式)
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_eff * K.transpose();
    
    // 8. 误差状态注入主状态
    injectErrorState(error_state_);
    
    // 9. 重置误差状态
    Eigen::Vector3d delta_theta = error_state_.segment<3>(StateIndex::DTHETA);
    Eigen::MatrixXd G = buildErrorResetMatrix(delta_theta);
    resetErrorState(G);
    
    return true;
}

bool EskfCore::updateWithDepth(const DepthData& depth_data) {
    if (!initialized_) {
        std::cerr << "改进ESKF深度更新失败: 滤波器未初始化!" << std::endl;
        return false;
    }
    
    // 1. 构建观测模型
    // h(x) = -p_z (深度 = -z坐标，因为z轴向下为正时深度为正)
    // H = ∂h/∂δx
    Eigen::MatrixXd H = buildDepthObservationMatrix(); // 1x15矩阵
    
    // 2. 计算观测残差 (ENU坐标系：Z朝上，深度为正值)
    // h(x) = -p_z (深度 = -z坐标，因为ENU中z朝上，深度朝下)
    // z = depth_measured - h(x_nominal) = depth_measured - (-p_z) = depth_measured + p_z
    double predicted_depth = -nominal_state_.position.z();
    double innovation = depth_data.depth - predicted_depth;
    
    // 3. 观测噪声方差（圆周运动时减小噪声，增强深度约束）
    double R = noise_params_.depth_noise_std * noise_params_.depth_noise_std;
    
    // 使用标准观测噪声
    
    // 4. 计算卡尔曼增益
    Eigen::MatrixXd PHt = P_ * H.transpose();
    Eigen::MatrixXd S = H * P_ * H.transpose();
    double R_eff = R;
    double innovation_used = innovation;

    // 初次求解S
    S(0,0) += R_eff;
    Eigen::LLT<Eigen::MatrixXd> llt_solver(S);
    if (llt_solver.info() != Eigen::Success) {
        std::cerr << "深度新息协方差矩阵不可逆!" << std::endl;
        return false;
    }
    
    // 正常更新
    Eigen::MatrixXd K = llt_solver.solve(PHt.transpose()).transpose();
    
    Eigen::VectorXd innovation_vec(1);
    innovation_vec(0) = innovation;
    Eigen::VectorXd delta_error = K * innovation_vec;
    error_state_ = delta_error;
    
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    
    // 7. 误差状态注入主状态
    injectErrorState(error_state_);
    
    // 8. 重置误差状态
    Eigen::Vector3d delta_theta = error_state_.segment<3>(StateIndex::DTHETA);
    Eigen::MatrixXd G = buildErrorResetMatrix(delta_theta);
    resetErrorState(G);
    
    return true;
}

static inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

bool EskfCore::updateWithHeading(const HeadingData& heading_data) {
    if (!initialized_) {
        std::cerr << "改进ESKF航向更新失败: 滤波器未初始化!" << std::endl;
        return false;
    }

    // 1. 观测模型：z = ψ_meas - ψ_pred，ψ_pred = yaw(q)
    // 从四元数提取yaw（ZYX顺序）
    const Eigen::Quaterniond& q = nominal_state_.orientation;
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw_pred = std::atan2(siny_cosp, cosy_cosp);

    double innovation = normalizeAngle(heading_data.yaw - yaw_pred);

    // 2. 雅可比 H：∂ψ/∂δθ ≈ [0, 0, 1]（小角度近似，yaw对z轴小转角最敏感）
    Eigen::MatrixXd H = buildHeadingObservationMatrix(); // 1x15

    // 3. 观测噪声方差
    double R = heading_data.variance;

    // 4. 计算卡尔曼增益并门限
    Eigen::MatrixXd S = H * P_ * H.transpose();
    S(0,0) += R;
    Eigen::MatrixXd PHt = P_ * H.transpose();
    Eigen::LLT<Eigen::MatrixXd> llt_solver(S);
    if (llt_solver.info() != Eigen::Success) {
        std::cerr << "航向新息协方差矩阵不可逆!" << std::endl;
        return false;
    }
    // 正常更新
    Eigen::MatrixXd K = llt_solver.solve(PHt.transpose()).transpose();

    // 5. 更新
    Eigen::VectorXd innovation_vec(1);
    innovation_vec(0) = innovation;
    Eigen::VectorXd delta_error = K * innovation_vec;
    error_state_ = delta_error;

    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // 6. 注入与重置
    injectErrorState(error_state_);
    Eigen::Vector3d delta_theta = error_state_.segment<3>(StateIndex::DTHETA);
    Eigen::MatrixXd G = buildErrorResetMatrix(delta_theta);
    resetErrorState(G);

    return true;
}

Eigen::MatrixXd EskfCore::buildDvlObservationMatrix() {
    // DVL观测在body坐标系，观测模型: h(x) = R_nb * v
    // 雅可比矩阵: H = ∂h/∂δx = [ ∂(R_nb v)/∂δp, ∂(R_nb v)/∂δv, ∂(R_nb v)/∂δθ, 0, 0 ]
    // 其中：
    //   ∂/∂δp = 0
    //   ∂/∂δv = R_nb
    //   ∂/∂δθ = [R_nb * v_n]×   （右乘、机体系误差）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_SIZE);
    
    Eigen::Matrix3d R_bn = nominal_state_.orientation.toRotationMatrix();
    Eigen::Matrix3d R_nb = R_bn.transpose();
    
    // 速度对误差速度的偏导
    H.block<3,3>(0, StateIndex::DV) = R_nb;
    
    // 对姿态误差的偏导: [R_nb * v_n]_x = R_nb * [v_n]_x * R_nb^T
    // 推导：右乘、机体系误差下，h = Exp(-δθ_b) R_nb v ≈ R_nb v - [δθ_b]_x (R_nb v)
    //      δh = [R_nb v]_x δθ_b
    Eigen::Matrix3d v_skew = skewSymmetric(nominal_state_.velocity);
    H.block<3,3>(0, StateIndex::DTHETA) = R_nb * v_skew * R_nb.transpose();
    
    return H;
}

Eigen::MatrixXd EskfCore::buildDepthObservationMatrix() {
    // 深度传感器观测模型: h(x) = -p_z (深度 = -z坐标)
    // 雅可比矩阵: H = ∂h/∂δx = [0, 0, -1, 0, ...] (只对z位置误差敏感)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_SIZE);
    
    // ∂(-p_z)/∂δp_z = -1
    H(0, StateIndex::DP + 2) = -1.0;  // z坐标在位置误差的第3个分量
    
    return H;
}

Eigen::MatrixXd EskfCore::buildHeadingObservationMatrix() {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_SIZE);
    // 小角度线性化下，yaw对δθ的灵敏度主要在z轴
    H(0, StateIndex::DTHETA + 2) = 1.0;
    return H;
}

void EskfCore::injectErrorState(const Eigen::VectorXd& error_state) {
    // 1. 位置误差注入
    nominal_state_.position += error_state.segment<3>(StateIndex::DP);
    
    // 2. 速度误差注入
    nominal_state_.velocity += error_state.segment<3>(StateIndex::DV);
    
    // 3. 姿态误差注入 (四元数复合)
    Eigen::Vector3d delta_theta = error_state.segment<3>(StateIndex::DTHETA);
    double angle = delta_theta.norm();
    
    if (angle > 1e-8) {
        // 使用精确的轴角转四元数
        Eigen::Vector3d axis = delta_theta / angle;
        double half_angle = 0.5 * angle;
        Eigen::Quaterniond dq(std::cos(half_angle), 
                             std::sin(half_angle) * axis.x(),
                             std::sin(half_angle) * axis.y(), 
                             std::sin(half_angle) * axis.z());
        // 右乘误差模型: q_true = q_nominal ⊗ δq (与雅可比矩阵推导一致)
        nominal_state_.orientation = nominal_state_.orientation * dq;
    } else {
        // 小角度近似
        Eigen::Quaterniond dq(1.0, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
        nominal_state_.orientation = nominal_state_.orientation * dq;
    }
    nominal_state_.orientation.normalize();
    
    // 4. 偏差误差注入
    Eigen::Vector3d new_gyro_bias = nominal_state_.gyro_bias + error_state.segment<3>(StateIndex::DBG);
    Eigen::Vector3d new_accel_bias = nominal_state_.accel_bias + error_state.segment<3>(StateIndex::DBA);
    
    nominal_state_.gyro_bias = new_gyro_bias;
    nominal_state_.accel_bias = new_accel_bias;
}

void EskfCore::resetErrorState(const Eigen::MatrixXd& G) {
    // 重置误差状态为0
    error_state_.setZero();
    
    // 更新协方差矩阵: P = G * P * G^T
    P_ = G * P_ * G.transpose();
}

Eigen::MatrixXd EskfCore::buildErrorResetMatrix(const Eigen::Vector3d& delta_theta) {
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    
    // 只有姿态误差需要特殊处理，其他保持单位矩阵
    double angle = delta_theta.norm();
    
    if (angle > 1e-8) {
        // 使用Rodrigues公式的一阶近似
        Eigen::Matrix3d G_theta = Eigen::Matrix3d::Identity() - 0.5 * skewSymmetric(delta_theta);
        G.block<3,3>(StateIndex::DTHETA, StateIndex::DTHETA) = G_theta;
    }
    
    return G;
}

Eigen::MatrixXd EskfCore::buildStateTransitionMatrix(double dt, const ImuData& raw_imu_data, const NominalState& previous_state) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    
    // 使用前一时刻的姿态矩阵
    Eigen::Matrix3d R_bn_prev = previous_state.orientation.toRotationMatrix();
    
    // 使用去偏后的IMU测量值
    Eigen::Vector3d omega_unbiased = raw_imu_data.angular_velocity - previous_state.gyro_bias;
    Eigen::Vector3d accel_unbiased = raw_imu_data.linear_acceleration - previous_state.accel_bias;
    Eigen::Matrix3d accel_skew = skewSymmetric(accel_unbiased);
    Eigen::Matrix3d omega_skew = skewSymmetric(omega_unbiased);
    
    // 填充状态转移矩阵 (ESKF误差状态传播模型)
    // δp = δp + δv * dt
    F.block<3,3>(StateIndex::DP, StateIndex::DV) = Eigen::Matrix3d::Identity() * dt;
    
    // δv = δv + (-R_bn * [a_unbiased]× * δθ - R_bn * δba) * dt
    // 使用去偏后的加速度值计算状态转移矩阵
    F.block<3,3>(StateIndex::DV, StateIndex::DTHETA) = -R_bn_prev * accel_skew * dt;
    F.block<3,3>(StateIndex::DV, StateIndex::DBA) = -R_bn_prev * dt;
    
    // δθ = δθ - [ω_unbiased]× * δθ * dt - δbg * dt
    // 使用去偏后的角速度构建状态转移矩阵（关键修复！）
    F.block<3,3>(StateIndex::DTHETA, StateIndex::DTHETA) = Eigen::Matrix3d::Identity() - omega_skew * dt;
    F.block<3,3>(StateIndex::DTHETA, StateIndex::DBG) = -Eigen::Matrix3d::Identity() * dt;
    
    // 偏差项保持不变 (随机游走模型)
    // δbg = δbg
    // δba = δba
    
    return F;
}

Eigen::MatrixXd EskfCore::buildProcessNoiseMatrix(double dt, const NominalState& previous_state) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    
    // 使用前一时刻的姿态旋转矩阵
    Eigen::Matrix3d R = previous_state.orientation.toRotationMatrix();
    
    // 构建离散时间过程噪声矩阵
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    double dt5 = dt4 * dt;
    
    // IMU噪声方差
    double gyro_var = noise_params_.gyro_noise_std * noise_params_.gyro_noise_std;
    double accel_var = noise_params_.accel_noise_std * noise_params_.accel_noise_std;
    double gyro_bias_var = noise_params_.gyro_bias_std * noise_params_.gyro_bias_std;  
    double accel_bias_var = noise_params_.accel_bias_std * noise_params_.accel_bias_std;
    
    // 使用标准偏差噪声参数
    
    // 位置噪声 (由速度噪声积分得到)
    Q.block<3,3>(StateIndex::DP, StateIndex::DP) = R * Eigen::Matrix3d::Identity() * R.transpose() * accel_var * dt3 / 3.0;
    Q.block<3,3>(StateIndex::DP, StateIndex::DV) = R * Eigen::Matrix3d::Identity() * R.transpose() * accel_var * dt2 / 2.0;
    Q.block<3,3>(StateIndex::DV, StateIndex::DP) = Q.block<3,3>(StateIndex::DP, StateIndex::DV).transpose();
    
    // 速度噪声 (由加速度噪声得到)
    Q.block<3,3>(StateIndex::DV, StateIndex::DV) = R * Eigen::Matrix3d::Identity() * R.transpose() * accel_var * dt;
    
    // 姿态噪声 (由角速度噪声得到)
    Q.block<3,3>(StateIndex::DTHETA, StateIndex::DTHETA) = Eigen::Matrix3d::Identity() * gyro_var * dt;
    
    // 陀螺偏差噪声 (随机游走)
    Q.block<3,3>(StateIndex::DBG, StateIndex::DBG) = Eigen::Matrix3d::Identity() * gyro_bias_var * dt;
    
    // 加速度计偏差噪声 (随机游走)
    Q.block<3,3>(StateIndex::DBA, StateIndex::DBA) = Eigen::Matrix3d::Identity() * accel_bias_var * dt;
    
    return Q;
}

Eigen::Vector3d EskfCore::computeEarthRotationRate(double latitude) const {
    // 地球自转角速度在 ENU 中的分量: [0, W*cos(phi), W*sin(phi)]
    Eigen::Vector3d wie_enu;
    wie_enu << 0.0, WGS84_WIE * cos(latitude), WGS84_WIE * sin(latitude);
    return wie_enu;
}

Eigen::Vector3d EskfCore::computeNavigationFrameRate(
    const Eigen::Vector3d& velocity, 
    const Eigen::Vector3d& position) const {
    // 使用实时纬度(基于ENU北向位移)，输出 ENU 形式的 w_en
    double phi = computeRealtimeLatitudeRad(position);
    Eigen::Vector2d rmrn = computeEarthRadii(phi); // [M, N]
    double rmh = rmrn[0] + position[2];
    double rnh = rmrn[1] + position[2];
    double vE = velocity[0];
    double vN = velocity[1];
    Eigen::Vector3d w_en_enu;
    // ENU: [-v_N/(R_M+h), v_E/(R_N+h), v_E*tan(phi)/(R_N+h)]
    w_en_enu[0] = -vN / rmh;
    w_en_enu[1] =  vE / rnh;
    w_en_enu[2] =  vE * std::tan(phi) / rnh;
    return w_en_enu;
}

Eigen::Vector2d EskfCore::computeEarthRadii(double latitude) const {
    // 计算子午圈半径(M)和卯酉圈半径(N) (参考KF-GINS Earth::meridianPrimeVerticalRadius)
    double sin_lat = sin(latitude);
    double cos_lat = cos(latitude);
    double sin2_lat = sin_lat * sin_lat;
    
    // 第一偏心率平方
    double e2 = WGS84_E1;
    double temp = 1.0 - e2 * sin2_lat;
    double sqrt_temp = sqrt(temp);
    
    // 卯酉圈半径 N = a / sqrt(1 - e2 * sin²(lat))
    double N = WGS84_RA / sqrt_temp;
    
    // 子午圈半径 M = a * (1 - e2) / (1 - e2 * sin²(lat))^(3/2)
    double M = WGS84_RA * (1.0 - e2) / (temp * sqrt_temp);
    
    return Eigen::Vector2d(M, N);
}

Eigen::MatrixXd EskfCore::buildOwttObservationMatrix(const Eigen::Vector3d& unit_vec) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_SIZE);
    H(0, StateIndex::DP + 0) = unit_vec.x();
    H(0, StateIndex::DP + 1) = unit_vec.y();
    H(0, StateIndex::DP + 2) = unit_vec.z();
    return H;
}

bool EskfCore::updateWithOwttRange(const OwttData& owtt_data) {
    if (!initialized_) {
        std::cerr << "改进ESKF OWTT更新失败: 滤波器未初始化!" << std::endl;
        return false;
    }

    // 预测距离和单位方向向量
    const Eigen::Vector3d& p_i = nominal_state_.position;
    Eigen::Vector3d diff = p_i - owtt_data.tx_position;
    double r_pred = diff.norm();
    if (r_pred < 1e-6) {
        std::cerr << "OWTT更新失败: 预测距离过小" << std::endl;
        return false;
    }
    Eigen::Vector3d u = diff / r_pred; // 从tx指向self

    // 残差 z - h(x)
    double innovation = owtt_data.range - r_pred;

    // 雅可比 H_i (1x15)，仅对自身位置误差敏感: ∂h/∂δp_i = u^T
    Eigen::MatrixXd H_i = buildOwttObservationMatrix(u);

    // 方法三：构造紧凑的 S 与 K_aug 所需项
    // R_eff 基础部分 = 传感器方差
    double R_base = owtt_data.variance;

    // 发送端位置不确定性投影项 u^T P_{p_j,p_j} u
    double proj_tx = u.transpose() * owtt_data.tx_position_covariance * u;

    // 局部自有项 H_i P_{ii} H_i^T
    double Si_local = (H_i * P_ * H_i.transpose())(0,0);

    // 互相关修正项：利用本地维护的 P_{x_i, p_j}
    Eigen::Matrix<double, STATE_SIZE, 3> P_xi_pj = Eigen::Matrix<double, STATE_SIZE, 3>::Zero();
    auto it = cross_cov_xi_p_peer_.find(owtt_data.peer_ns);
    if (it != cross_cov_xi_p_peer_.end())
        P_xi_pj = it->second;

    // 计算 P_aug H_aug^T = P_{:,p_i} * u + P_{:,p_j} * (-u)
    // 这里 P_{:,p_i} 即 P_ 对应 δp_i 的列块；P_{:,p_j} 用 P_{x_i,p_j}
    Eigen::Matrix<double, STATE_SIZE, 1> P_aug_Ht = P_ * H_i.transpose(); // = P_{:,p_i} * u
    // 加上对端项（注意 H_j = -u^T 对 δp_j）
    P_aug_Ht -= P_xi_pj * u;

    // 交叉项 u^T P_{p_i,p_j} u（由本地维护的 P_{x_i,p_j} 的位置块得到）
    Eigen::Matrix3d P_pi_pj = P_xi_pj.block<3,3>(StateIndex::DP, 0);
    double proj_cross = u.transpose() * P_pi_pj * u;
    // 新息协方差 S = u^T P_{p_i,p_i} u + u^T P_{p_j,p_j} u - 2 u^T P_{p_i,p_j} u + R
    double S_scalar = Si_local + proj_tx - 2.0 * proj_cross + R_base;
    if (!(S_scalar > 0)) S_scalar = R_base > 0 ? R_base : 1.0;

    // 增益前向量（对应增广 K 的本地部分）
    Eigen::Matrix<double, STATE_SIZE, 1> K_local = P_aug_Ht / S_scalar;

    // 更新本地误差状态
    Eigen::VectorXd delta_error = K_local * innovation; // 15x1
    error_state_ = delta_error;

    // 协方差更新（Joseph）：P_new = P - K_local * S * K_local^T，其中 S 是标量
    P_ = P_ - (K_local * S_scalar) * K_local.transpose();
    // 对称化
    P_ = 0.5 * (P_ + P_.transpose());

    // 注入与重置
    injectErrorState(error_state_);
    Eigen::Vector3d delta_theta = error_state_.segment<3>(StateIndex::DTHETA);
    Eigen::MatrixXd G = buildErrorResetMatrix(delta_theta);
    resetErrorState(G);

    // 互相关更新（维护 P_{x_i,p_j}）: P_{x_i,p_j,new} = P_{x_i,p_j} - K_local * (H_i P_{x_i,p_j} - u^T P_{p_j,p_j})
    // 推导自方法三的增广协方差更新在位置列块上的等价形式
    // 首先计算 h_i_pj = H_i * P_{x_i,p_j}，这是 1x3 行向量
    Eigen::RowVector3d h_i_pj = (H_i * P_xi_pj).row(0);
    // 等价右项（1x3）: h_i_pj - u^T P_{p_j,p_j}
    Eigen::RowVector3d rhs = h_i_pj - (u.transpose() * owtt_data.tx_position_covariance);
    // K_local (15x1) * rhs (1x3) => (15x3)
    Eigen::Matrix<double, STATE_SIZE, 3> delta_cross = K_local * rhs;
    Eigen::Matrix<double, STATE_SIZE, 3> P_xi_pj_new = P_xi_pj - delta_cross;
    // 与本地误差状态重置保持一致：P_{x_i,p_j} <- G * P_{x_i,p_j}
    P_xi_pj_new = G * P_xi_pj_new;
    cross_cov_xi_p_peer_[owtt_data.peer_ns] = P_xi_pj_new;

    return true;
}

} // namespace uuv_eskf_nav
