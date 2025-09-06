#ifndef UUV_ESKF_NAV_ESKF_CORE_H
#define UUV_ESKF_NAV_ESKF_CORE_H

#include "eskf_types.h"

namespace uuv_eskf_nav {

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

private:
    /**
     * @brief 改进的姿态更新 - 基于KF-GINS方法
     * @param imu_curr 当前积分IMU数据
     * @param imu_prev 前一积分IMU数据
     * @return 更新后的姿态四元数
     */
    Eigen::Quaterniond improvedAttitudeUpdate(
        const ImuIntegralData& imu_curr,
        const ImuIntegralData& imu_prev);

    /**
     * @brief 二阶圆锥误差补偿
     * @param delta_theta_curr 当前角增量
     * @param delta_theta_prev 前一角增量  
     * @return 补偿后的等效旋转向量
     */
    Eigen::Vector3d coningCorrection(
        const Eigen::Vector3d& delta_theta_curr,
        const Eigen::Vector3d& delta_theta_prev);

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
};

} // namespace uuv_eskf_nav

#endif // UUV_ESKF_NAV_ESKF_CORE_H
