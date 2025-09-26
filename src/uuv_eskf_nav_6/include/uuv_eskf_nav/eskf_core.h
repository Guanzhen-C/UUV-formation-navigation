#ifndef UUV_ESKF_NAV_ESKF_CORE_H
#define UUV_ESKF_NAV_ESKF_CORE_H

#include "eskf_types.h"
#include <unordered_map>
#include <string>

namespace uuv_eskf_nav {

// WGS84椭球模型参数 (参考KF-GINS)
const double WGS84_WIE = 7.2921151467E-5;       // 地球自转角速度 [rad/s]
const double WGS84_RA  = 6378137.0000000000;    // 长半轴a [m]
const double WGS84_RB  = 6356752.3142451793;    // 短半轴b [m]
const double WGS84_E1  = 0.0066943799901413156; // 第一偏心率平方

/**
 * @brief 改进的ESKF核心算法 - 基于KF-GINS标准机械编排
 * 
 * 主要改进：
 * 1. 将瞬时IMU数据转换为角增量/速度增量
 * 2. 实现二阶圆锥误差补偿  
 * 3. 考虑导航系转动效应（简化版）
 * 4. 更严格的数值积分方法
 */
/**
 * @brief IMU积分数据（模拟真实IMU输出）
 */
struct ImuIntegralData {
    Eigen::Vector3d delta_theta;    // 角增量 [rad]
    Eigen::Vector3d delta_velocity; // 速度增量 [m/s]
    double dt;                      // 积分时间间隔 [s]
    double timestamp;               // 时间戳
    
    // 从瞬时IMU数据转换而来
    static ImuIntegralData fromInstantaneous(
        const ImuData& imu_current,
        const ImuData& imu_previous,
        double dt);
};

class EskfCore {
public:

    explicit EskfCore(const NoiseParams& noise_params, bool accel_includes_gravity = false);
    
    /**
     * @brief 初始化改进ESKF
     * @param initial_state 初始状态
     * @param initial_covariance 初始协方差矩阵
     * @return true if successful
     */
    bool initialize(const NominalState& initial_state,
                   const Eigen::MatrixXd& initial_covariance);
    
    /**
     * @brief 获取当前状态估计
     * @return 当前主状态
     */
    const NominalState& getNominalState() const { return nominal_state_; }
    
    /**
     * @brief 改进的IMU预测步骤
     * @param imu_current 当前时刻IMU数据
     * @param imu_previous 前一时刻IMU数据  
     * @return true if successful
     */
    bool predictWithImprovedMechanization(
        const ImuData& imu_current,
        const ImuData& imu_previous);

    /**
     * @brief DVL速度量测更新
     * @param dvl_data DVL测量数据
     * @return true if successful
     */
    bool updateWithDvl(const DvlData& dvl_data);

    /**
     * @brief 深度传感器量测更新
     * @param depth_data 深度测量数据
     * @return true if successful
     */
    bool updateWithDepth(const DepthData& depth_data);

    /**
     * @brief 航向角量测更新（例如磁罗盘/IMU yaw）
     */
    bool updateWithHeading(const HeadingData& heading_data);

    /**
     * @brief OWTT单程测距量测更新
     */
    bool updateWithOwttRange(const OwttData& owtt_data);

    /**
     * @brief 获取当前误差状态协方差
     * @return 15x15协方差矩阵
     */
    const Eigen::MatrixXd& getCovariance() const { return P_; }

    /**
     * @brief 检查滤波器是否已初始化
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief 设置IMU加速度是否包含重力 (true表示包含重力，需要在推进中不再加g)
     */
    void setAccelIncludesGravity(bool flag) { accel_includes_gravity_ = flag; }

    /**
     * @brief 设置IMU到base_link的固定外参旋转矩阵 (R_imu_to_base)
     */
    void setImuToBaseRotation(const Eigen::Matrix3d& R_imu_to_base) { R_imu_to_base_ = R_imu_to_base; }

    /**
     * @brief 设置地球自转补偿开关和任务纬度
     * @param enable_earth_rotation 是否启用地球自转补偿
     * @param latitude_rad 任务区域纬度 [弧度]
     */
    void setEarthRotationParams(bool enable_earth_rotation, double latitude_rad = 0.3183) {
        enable_earth_rotation_ = enable_earth_rotation;
        mission_latitude_ = latitude_rad;
    }

    // 测试函数访问权限
    Eigen::Vector3d computeEarthRotationRate(double latitude) const;
    Eigen::Vector2d computeEarthRadii(double latitude) const;
    Eigen::Vector3d computeNavigationFrameRate(const Eigen::Vector3d& velocity, const Eigen::Vector3d& position) const;

private:
    // 维护本端误差状态与各对端位置的互协方差(15x3)
    std::unordered_map<std::string, Eigen::Matrix<double, STATE_SIZE, 3>> cross_cov_xi_p_peer_;
    /**
     * @brief 速度更新 - 基于KF-GINS velUpdate
     * @param state_prev 前一时刻状态
     * @param velocity_curr 输出的当前速度
     * @param imu_integral 当前IMU积分数据
     * @param dt 时间间隔
     */
    void velocityUpdate(
        const NominalState& state_prev,
        Eigen::Vector3d& velocity_curr,
        const ImuIntegralData& imu_integral,
        double dt);
    
    /**
     * @brief 位置更新 - 基于KF-GINS posUpdate
     * @param state_prev 前一时刻状态
     * @param position_curr 输出的当前位置
     * @param velocity_curr 当前速度
     * @param dt 时间间隔
     */
    void positionUpdate(
        const NominalState& state_prev,
        Eigen::Vector3d& position_curr,
        const Eigen::Vector3d& velocity_curr,
        double dt);
    
    /**
     * @brief 姿态更新 - 基于KF-GINS attUpdate
     * @param state_prev 前一时刻状态
     * @param orientation_curr 输出的当前姿态
     * @param velocity_curr 当前速度
     * @param position_curr 当前位置
     * @param imu_integral 当前IMU积分数据
     * @param dt 时间间隔
     */
    void attitudeUpdate(
        const NominalState& state_prev,
        Eigen::Quaterniond& orientation_curr,
        const Eigen::Vector3d& velocity_curr,
        const Eigen::Vector3d& position_curr,
        const ImuIntegralData& imu_integral,
        double dt);

    /**
     * @brief 旋转和划桨效应补偿
     * @param delta_theta 角增量
     * @param delta_vel 速度增量
     * @return 补偿后的速度增量
     */
    Eigen::Vector3d scullingCorrection(
        const Eigen::Vector3d& delta_theta,
        const Eigen::Vector3d& delta_vel);

    /**
     * @brief 简化的导航系转动效应（用于水下短时导航）
     * @param velocity 当前速度
     * @param position 当前位置  
     * @param dt 时间间隔
     * @return 导航系转动四元数
     */
    Eigen::Quaterniond navigationFrameRotation(
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& position, 
        double dt);

    /**
     * @brief 旋转向量转四元数（Rodrigues公式）
     * @param rotation_vector 旋转向量 [rad]
     * @return 单位四元数
     */
    Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d& rotation_vector);


    /**
     * @brief 构建DVL观测模型的雅可比矩阵
     * @return 3x15观测矩阵
     */
    Eigen::MatrixXd buildDvlObservationMatrix();

    /**
     * @brief 构建深度观测模型的雅可比矩阵
     * @return 1x15观测矩阵
     */
    Eigen::MatrixXd buildDepthObservationMatrix();

    /**
     * @brief 构建航向角观测雅可比（1x15）
     */
    Eigen::MatrixXd buildHeadingObservationMatrix();

    /**
     * @brief 构建OWTT观测雅可比（1x15），仅对位置误差敏感
     */
    Eigen::MatrixXd buildOwttObservationMatrix(const Eigen::Vector3d& unit_vec);

    /**
     * @brief 应用误差状态到主状态 (误差状态注入)
     * @param error_state 15维误差状态
     */
    void injectErrorState(const Eigen::VectorXd& error_state);

    /**
     * @brief 误差状态注入后重置误差状态
     * @param G 误差重置矩阵
     */
    void resetErrorState(const Eigen::MatrixXd& G);

    /**
     * @brief 构建误差重置矩阵G (用于姿态误差重置)
     * @param delta_theta 姿态误差角
     * @return 15x15重置矩阵
     */
    Eigen::MatrixXd buildErrorResetMatrix(const Eigen::Vector3d& delta_theta);

    /**
     * @brief 构建状态转移矩阵 F (修复：基于前一时刻状态和真实测量值)
     * @param dt 时间间隔
     * @param raw_imu_data 原始IMU数据（含偏差）
     * @param previous_state 前一时刻的主状态
     * @return 15x15状态转移矩阵
     */
    Eigen::MatrixXd buildStateTransitionMatrix(double dt, const ImuData& raw_imu_data, const NominalState& previous_state);

    /**
     * @brief 构建过程噪声矩阵 Q (修复：基于前一时刻状态)
     * @param dt 时间间隔
     * @param previous_state 前一时刻的主状态
     * @return 15x15过程噪声协方差矩阵
     */
    Eigen::MatrixXd buildProcessNoiseMatrix(double dt, const NominalState& previous_state);

    // 存储前一时刻的数据用于圆锥补偿
    ImuIntegralData previous_imu_integral_;
    bool has_previous_imu_;
    
    // 基础ESKF功能（继承原有接口）
    NoiseParams noise_params_;
    NominalState nominal_state_;
    Eigen::VectorXd error_state_;
    Eigen::MatrixXd P_;
    bool initialized_;
    bool accel_includes_gravity_;
    Eigen::Matrix3d R_imu_to_base_ = Eigen::Matrix3d::Identity();
    
    // 地球自转补偿参数
    bool enable_earth_rotation_;
    double mission_latitude_;  // 任务区域纬度 [弧度]
    // 根据ENU北向位移实时估计纬度
    double computeRealtimeLatitudeRad(const Eigen::Vector3d& enu_position) const;
};

} // namespace uuv_eskf_nav

#endif // UUV_ESKF_NAV_ESKF_CORE_H
