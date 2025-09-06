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
    , accel_includes_gravity_(accel_includes_gravity) {
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
    
    // 转换为积分数据格式（使用补偿后的数据）
    ImuIntegralData imu_curr_integral = ImuIntegralData::fromInstantaneous(
        corrected_current, corrected_previous, dt);
    
    // 1. 改进的姿态更新（包含圆锥误差补偿 + 简化导航系转动）
    Eigen::Quaterniond new_orientation;
    if (has_previous_imu_) {
        new_orientation = improvedAttitudeUpdate(imu_curr_integral, previous_imu_integral_);
    } else {
        // 第一帧，使用简单方法
        Eigen::Vector3d rotation_vector = imu_curr_integral.delta_theta;
        Eigen::Quaterniond dq = rotationVectorToQuaternion(rotation_vector);
        new_orientation = (nominal_state_.orientation * dq).normalized();
    }
    
    // 2. 速度更新（修复机械编排顺序）
    Eigen::Vector3d compensated_delta_vel = imu_curr_integral.delta_velocity;
    if (has_previous_imu_) {
        compensated_delta_vel += scullingCorrection(
            imu_curr_integral.delta_theta, 
            imu_curr_integral.delta_velocity);
    }
    
    // 修复：正确的机械编排 - 比力投影后加上重力
    // 使用名义与更新后姿态的平均旋转近似，降低时序误差
    Eigen::Quaterniond q_old = nominal_state_.orientation;
    Eigen::Quaterniond q_new = new_orientation;
    Eigen::Quaterniond q_avg = q_old.slerp(0.5, q_new);
    Eigen::Matrix3d R_bn = q_avg.toRotationMatrix();
    
    // 载体系比力投影到导航系
    Eigen::Vector3d delta_v_n = R_bn * compensated_delta_vel;
    
    // 根据配置决定是否添加重力
    Eigen::Vector3d gravity_term;
    if (accel_includes_gravity_) {
        gravity_term.setZero();
    } else {
        gravity_term = GRAVITY_ENU * dt;
    }
    Eigen::Vector3d new_velocity = nominal_state_.velocity + delta_v_n + gravity_term;
    
    // 3. 位置更新（梯形积分）
    Eigen::Vector3d avg_velocity = 0.5 * (nominal_state_.velocity + new_velocity);
    Eigen::Vector3d new_position = nominal_state_.position + avg_velocity * dt;
    
    // 4. 偏差保持不变（随机游走模型）
    Eigen::Vector3d new_gyro_bias = nominal_state_.gyro_bias;
    Eigen::Vector3d new_accel_bias = nominal_state_.accel_bias;
    
    // 5. 协方差预测 (修复：在状态更新前基于前一时刻状态)
    // 构建状态转移矩阵和过程噪声矩阵 - 使用真实测量值（含偏差）
    ImuData raw_imu_current = imu_current;  // 使用原始测量值
    
    // 保存前一时刻状态用于F和Q计算
    NominalState previous_state = nominal_state_;
    Eigen::MatrixXd F = buildStateTransitionMatrix(dt, raw_imu_current, previous_state);
    Eigen::MatrixXd Q = buildProcessNoiseMatrix(dt, previous_state);
    
    // 协方差预测: P = F * P * F^T + Q (标准卡尔曼滤波公式)
    P_ = F * P_ * F.transpose() + Q;
    
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
    
    std::cout << "改进ESKF预测完成，姿态四元数: [" 
              << new_orientation.w() << ", " << new_orientation.x() << ", " 
              << new_orientation.y() << ", " << new_orientation.z() << "]" << std::endl;
    
    return true;
}

Eigen::Quaterniond EskfCore::improvedAttitudeUpdate(
    const ImuIntegralData& imu_curr,
    const ImuIntegralData& imu_prev) {
    
    // 1. 二阶圆锥误差补偿（基于KF-GINS第181行）
    Eigen::Vector3d corrected_rotation = coningCorrection(
        imu_curr.delta_theta, 
        imu_prev.delta_theta);
    
    // 2. 载体系旋转四元数
    Eigen::Quaterniond q_bb = rotationVectorToQuaternion(corrected_rotation);
    
    // 3. 简化的导航系转动补偿（对于水下短时导航）
    Eigen::Quaterniond q_nn = navigationFrameRotation(
        nominal_state_.velocity, 
        nominal_state_.position, 
        imu_curr.dt);
    
    // 4. 完整的姿态更新：q_new = q_old * q_bb (载体系旋转)
    // 注意：这里简化了导航系效应，主要保留圆锥补偿
    // 正确顺序：先有名义姿态，再应用载体系旋转增量
    Eigen::Quaterniond new_orientation = (nominal_state_.orientation * q_bb).normalized();
    
    return new_orientation;
}

Eigen::Vector3d EskfCore::coningCorrection(
    const Eigen::Vector3d& delta_theta_curr,
    const Eigen::Vector3d& delta_theta_prev) {
    
    // KF-GINS标准二阶圆锥误差补偿公式（第181行）
    // temp1 = imucur.dtheta + imupre.dtheta.cross(imucur.dtheta) / 12
    Eigen::Vector3d coning_correction = delta_theta_prev.cross(delta_theta_curr) / 12.0;
    
    Eigen::Vector3d corrected_rotation = delta_theta_curr + coning_correction;
    
    // 输出补偿量用于调试
    if (coning_correction.norm() > 1e-6) {
        std::cout << "圆锥误差补偿: " << coning_correction.transpose() * 180.0 / M_PI 
                  << " [mdeg]" << std::endl;
    }
    
    return corrected_rotation;
}

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
    
    if (sculling_correction.norm() > 1e-6) {
        std::cout << "划桨效应补偿: " << sculling_correction.transpose() 
                  << " [mm/s]" << std::endl;
    }
    
    return sculling_correction;
}

Eigen::Quaterniond EskfCore::navigationFrameRotation(
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& position,
    double dt) {
    
    // 简化的导航系转动（适用于水下短距离导航）
    // 完整版本需要地球椭球参数和重力场模型
    
    // 地球自转角速度 (简化，只考虑Z轴分量)
    double omega_ie = 7.2921151467e-5; // [rad/s]
    
    // 简化的导航系转动角速度（忽略高阶项）
    Eigen::Vector3d omega_en_simplified;
    omega_en_simplified << 0, 0, -omega_ie;
    
    // 导航系转动角增量
    Eigen::Vector3d nav_rotation = omega_en_simplified * dt;
    
    // 转换为四元数（取负号因为是n(k-1)相对n(k)的旋转）
    return rotationVectorToQuaternion(-nav_rotation);
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
    
    std::cout << "执行改进ESKF DVL量测更新..." << std::endl;
    
    // 1. 构建观测模型（在body坐标系中进行对比）
    // 观测模型: h(x) = R_nb * v_nominal，其中R_nb = R_bn^T
    // 注意：DVL速度已在SensorManager中转换至base_link坐标系
    Eigen::Matrix3d R_bn = nominal_state_.orientation.toRotationMatrix();
    Eigen::Matrix3d R_nb = R_bn.transpose();
    
    // 2. 计算观测残差 (新息)
    // z = v_body_measured - R_nb * v_nav
    Eigen::Vector3d predicted_body_velocity = R_nb * nominal_state_.velocity;
    Eigen::Vector3d innovation = dvl_data.velocity - predicted_body_velocity;
    
    // 3. 动态构建观测雅可比矩阵 H
    Eigen::MatrixXd H = buildDvlObservationMatrix(); // 3x15矩阵，内部基于当前名义状态构建
    
    std::cout << "DVL观测残差: [" << innovation.transpose() << "] m/s" << std::endl;
    
    // 4. 观测噪声协方差（与配置保持一致，忽略上游默认常量）
    Eigen::Matrix3d R_base = Eigen::Matrix3d::Identity() * (noise_params_.dvl_noise_std * noise_params_.dvl_noise_std);

    // 5. 计算卡尔曼增益，加入“软门限”：对于大残差，裁剪创新并自适应放大R，避免直接拒绝
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
    Eigen::Vector3d S_inv_innov = llt_solver.solve(innovation);
    double mahalanobis = innovation.dot(S_inv_innov);
    const double gate_hi = 18.5; // 更宽松的99.95%门限（dof=3）
    if (mahalanobis > gate_hi) {
        // 裁剪创新幅值，并放大R以降低置信度，从而允许更新将状态拉回
        double scale = std::sqrt(gate_hi / std::max(mahalanobis, 1e-9));
        innovation_used = innovation * scale;
        double inflate = 1.0 / (scale * scale);
        R_eff = R_base * inflate;

        // 以放大后的R重新计算增益
        S = H * P_ * H.transpose() + R_eff;
        Eigen::LLT<Eigen::MatrixXd> llt_solver2(S);
        if (llt_solver2.info() != Eigen::Success) {
            std::cerr << "DVL新息协方差矩阵不可逆(自适应阶段)!" << std::endl;
            return false;
        }
        Eigen::MatrixXd K = llt_solver2.solve(PHt.transpose()).transpose();

        // 6. 误差状态更新
        Eigen::VectorXd delta_error = K * innovation_used;
        error_state_ = delta_error;
        std::cout << "更新后误差状态范数(裁剪): " << error_state_.norm() << std::endl;

        // 7. 协方差更新 (Joseph形式)
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_eff * K.transpose();
    } else {
        // 正常更新路径
        Eigen::MatrixXd K = llt_solver.solve(PHt.transpose()).transpose();
        Eigen::VectorXd delta_error = K * innovation_used;
        error_state_ = delta_error;
        std::cout << "更新后误差状态范数: " << error_state_.norm() << std::endl;
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_eff * K.transpose();
    }
    
    // 8. 误差状态注入主状态
    injectErrorState(error_state_);
    
    // 9. 重置误差状态
    Eigen::Vector3d delta_theta = error_state_.segment<3>(StateIndex::DTHETA);
    Eigen::MatrixXd G = buildErrorResetMatrix(delta_theta);
    resetErrorState(G);
    
    std::cout << "✅ 改进ESKF DVL量测更新完成!" << std::endl;
    
    return true;
}

bool EskfCore::updateWithDepth(const DepthData& depth_data) {
    if (!initialized_) {
        std::cerr << "改进ESKF深度更新失败: 滤波器未初始化!" << std::endl;
        return false;
    }
    
    std::cout << "执行改进ESKF深度量测更新..." << std::endl;
    
    // 1. 构建观测模型
    // h(x) = -p_z (深度 = -z坐标，因为z轴向下为正时深度为正)
    // H = ∂h/∂δx
    Eigen::MatrixXd H = buildDepthObservationMatrix(); // 1x15矩阵
    
    // 2. 计算观测残差 (ENU坐标系：Z朝上，深度为正值)
    // h(x) = -p_z (深度 = -z坐标，因为ENU中z朝上，深度朝下)
    // z = depth_measured - h(x_nominal) = depth_measured - (-p_z) = depth_measured + p_z
    double predicted_depth = -nominal_state_.position.z();
    double innovation = depth_data.depth - predicted_depth;
    
    std::cout << "深度观测残差: " << innovation << " m" << std::endl;
    
    // 3. 观测噪声方差（与配置保持一致）
    double R = noise_params_.depth_noise_std * noise_params_.depth_noise_std;
    
    // 4. 计算卡尔曼增益 (修复：使用数值稳定的求解方法)
    Eigen::MatrixXd S = H * P_ * H.transpose();
    S(0,0) += R;
    Eigen::MatrixXd PHt = P_ * H.transpose();
    
    // 使用LLT分解求解，避免直接求逆
    Eigen::LLT<Eigen::MatrixXd> llt_solver(S);
    if (llt_solver.info() != Eigen::Success) {
        std::cerr << "深度新息协方差矩阵不可逆!" << std::endl;
        return false;
    }
    // 4.1 门限检测（1自由度）: 99%阈值≈6.635，95%≈3.841
    double S_scalar = S(0,0);
    double mahalanobis = (innovation * innovation) / S_scalar;
    if (mahalanobis > 6.635) {
        std::cerr << "深度量测被门限拒绝，马氏距离=" << mahalanobis << std::endl;
        return false;
    }
    Eigen::MatrixXd K = llt_solver.solve(PHt.transpose()).transpose();
    
    // 5. 误差状态更新
    Eigen::VectorXd innovation_vec(1);
    innovation_vec(0) = innovation;
    Eigen::VectorXd delta_error = K * innovation_vec;
    error_state_ = delta_error;
    
    // 6. 协方差更新
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    
    // 7. 误差状态注入主状态
    injectErrorState(error_state_);
    
    // 8. 重置误差状态
    Eigen::Vector3d delta_theta = error_state_.segment<3>(StateIndex::DTHETA);
    Eigen::MatrixXd G = buildErrorResetMatrix(delta_theta);
    resetErrorState(G);
    
    std::cout << "✅ 改进ESKF深度量测更新完成!" << std::endl;
    
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
    double S_scalar = S(0,0);
    double mahalanobis = (innovation * innovation) / S_scalar;
    if (mahalanobis > 6.635) {
        std::cerr << "航向量测被门限拒绝，马氏距离=" << mahalanobis << std::endl;
        return false;
    }
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
    //   ∂/∂δθ ≈ -R_nb * [v]×   (基于小角度近似与左乘误差注入)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_SIZE);
    
    Eigen::Matrix3d R_bn = nominal_state_.orientation.toRotationMatrix();
    Eigen::Matrix3d R_nb = R_bn.transpose();
    
    // 速度对误差速度的偏导
    H.block<3,3>(0, StateIndex::DV) = R_nb;
    
    // 对姿态误差的偏导: -R_nb * [v]_x  (左乘误差注入的一阶线性化)
    Eigen::Matrix3d v_skew = skewSymmetric(nominal_state_.velocity);
    H.block<3,3>(0, StateIndex::DTHETA) = -R_nb * v_skew;
    
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
    std::cout << "误差状态注入: 误差范数 = " << error_state.norm() << std::endl;
    
    // 1. 位置误差注入
    nominal_state_.position += error_state.segment<3>(StateIndex::DP);
    
    // 2. 速度误差注入
    nominal_state_.velocity += error_state.segment<3>(StateIndex::DV);
    
    // 3. 姿态误差注入 (四元数复合) - 关键改进点！
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
        // 修复：正确的四元数乘法顺序 q_true = δq ⊗ q_nominal
        nominal_state_.orientation = dq * nominal_state_.orientation;
    } else {
        // 小角度近似
        Eigen::Quaterniond dq(1.0, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
        // 修复：正确的四元数乘法顺序
        nominal_state_.orientation = dq * nominal_state_.orientation;
    }
    nominal_state_.orientation.normalize();
    
    // 4. 偏差误差注入
    nominal_state_.gyro_bias += error_state.segment<3>(StateIndex::DBG);
    nominal_state_.accel_bias += error_state.segment<3>(StateIndex::DBA);
    
    std::cout << "主状态更新后位置: [" << nominal_state_.position.transpose() << "]" << std::endl;
}

void EskfCore::resetErrorState(const Eigen::MatrixXd& G) {
    // 重置误差状态为0
    error_state_.setZero();
    
    // 更新协方差矩阵: P = G * P * G^T
    P_ = G * P_ * G.transpose();
    
    std::cout << "误差状态已重置, 协方差矩阵迹: " << P_.trace() << std::endl;
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
    
    // 修复：使用前一时刻的姿态旋转矩阵
    Eigen::Matrix3d R = previous_state.orientation.toRotationMatrix();
    
    // 修复：使用去偏后的IMU测量值（specific force与角速度）
    Eigen::Vector3d omega_unbiased = raw_imu_data.angular_velocity - previous_state.gyro_bias;
    Eigen::Vector3d accel_unbiased = raw_imu_data.linear_acceleration - previous_state.accel_bias; // specific force
    Eigen::Matrix3d accel_skew = skewSymmetric(accel_unbiased);
    Eigen::Matrix3d omega_skew = skewSymmetric(omega_unbiased);
    
    // 填充状态转移矩阵 (ESKF误差状态传播模型)
    // δp = δp + δv * dt
    F.block<3,3>(StateIndex::DP, StateIndex::DV) = Eigen::Matrix3d::Identity() * dt;
    
    // δv = δv + (-R * [a_m]× * δθ - R * δba + [g]× * δθ) * dt
    // 其中a_m是含偏差的测量加速度，g是重力向量
    Eigen::Matrix3d gravity_skew = skewSymmetric(GRAVITY_ENU);
    F.block<3,3>(StateIndex::DV, StateIndex::DTHETA) = (-R * accel_skew + gravity_skew) * dt;
    F.block<3,3>(StateIndex::DV, StateIndex::DBA) = -R * dt;
    
    // δθ = δθ - [ω_m]× * δθ * dt - δbg * dt  
    // 其中ω_m是含偏差的测量角速度 (关键修复！)
    F.block<3,3>(StateIndex::DTHETA, StateIndex::DTHETA) = Eigen::Matrix3d::Identity() - omega_skew * dt;
    F.block<3,3>(StateIndex::DTHETA, StateIndex::DBG) = -Eigen::Matrix3d::Identity() * dt;
    
    // 偏差项保持不变 (随机游走模型)
    // δbg = δbg
    // δba = δba
    
    return F;
}

Eigen::MatrixXd EskfCore::buildProcessNoiseMatrix(double dt, const NominalState& previous_state) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    
    // 修复：使用前一时刻的姿态旋转矩阵
    Eigen::Matrix3d R = previous_state.orientation.toRotationMatrix();
    
    // 构建连续时间过程噪声功率谱密度矩阵
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    // IMU噪声方差
    double gyro_var = noise_params_.gyro_noise_std * noise_params_.gyro_noise_std;
    double accel_var = noise_params_.accel_noise_std * noise_params_.accel_noise_std;
    double gyro_bias_var = noise_params_.gyro_bias_std * noise_params_.gyro_bias_std;  
    double accel_bias_var = noise_params_.accel_bias_std * noise_params_.accel_bias_std;
    
    // 修复：正确的过程噪声矩阵构建（基于前一时刻姿态）
    // 位置噪声 (由加速度噪声积分得到)
    Q.block<3,3>(StateIndex::DP, StateIndex::DP) = R * Eigen::Matrix3d::Identity() * R.transpose() * accel_var * dt4 / 4.0;
    Q.block<3,3>(StateIndex::DP, StateIndex::DV) = R * Eigen::Matrix3d::Identity() * R.transpose() * accel_var * dt3 / 2.0;
    Q.block<3,3>(StateIndex::DV, StateIndex::DP) = Q.block<3,3>(StateIndex::DP, StateIndex::DV).transpose();
    
    // 速度噪声
    Q.block<3,3>(StateIndex::DV, StateIndex::DV) = R * Eigen::Matrix3d::Identity() * R.transpose() * accel_var * dt2;
    
    // 姿态噪声
    Q.block<3,3>(StateIndex::DTHETA, StateIndex::DTHETA) = Eigen::Matrix3d::Identity() * gyro_var * dt2;
    
    // 陀螺偏差噪声 
    Q.block<3,3>(StateIndex::DBG, StateIndex::DBG) = Eigen::Matrix3d::Identity() * gyro_bias_var * dt;
    
    // 加速度计偏差噪声
    Q.block<3,3>(StateIndex::DBA, StateIndex::DBA) = Eigen::Matrix3d::Identity() * accel_bias_var * dt;
    
    return Q;
}

} // namespace uuv_eskf_nav
