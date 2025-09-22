#ifndef UUV_ESKF_NAV_ESKF_TYPES_H
#define UUV_ESKF_NAV_ESKF_TYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace uuv_eskf_nav {

/**
 * @brief 误差状态卡尔曼滤波器的状态和误差状态定义
 * 
 * 主状态向量 (Nominal State):
 * - 位置: p (3D) - [x, y, z] in world frame
 * - 速度: v (3D) - [vx, vy, vz] in world frame  
 * - 姿态: q (quaternion) - body to world frame
 * - 陀螺仪偏差: bg (3D) - gyroscope bias
 * - 加速度计偏差: ba (3D) - accelerometer bias
 * 
 * 误差状态向量 (Error State) - 15维:
 * - 位置误差: δp (3D)
 * - 速度误差: δv (3D)  
 * - 姿态误差: δθ (3D) - rotation vector
 * - 陀螺仪偏差误差: δbg (3D)
 * - 加速度计偏差误差: δba (3D)
 */

// 状态维度定义
constexpr int STATE_SIZE = 15;
constexpr int NOMINAL_STATE_SIZE = 16; // 位置3 + 速度3 + 四元数4 + 陀螺偏差3 + 加速度偏差3

// 状态向量索引
namespace StateIndex {
    // 误差状态向量索引 (15维)
    constexpr int DP = 0;      // 位置误差 δp [0:2]
    constexpr int DV = 3;      // 速度误差 δv [3:5]
    constexpr int DTHETA = 6;  // 姿态误差 δθ [6:8]  
    constexpr int DBG = 9;     // 陀螺偏差误差 δbg [9:11]
    constexpr int DBA = 12;    // 加速度偏差误差 δba [12:14]
    
    // 主状态向量索引 (16维)
    constexpr int P = 0;       // 位置 p [0:2]
    constexpr int V = 3;       // 速度 v [3:5] 
    constexpr int Q = 6;       // 四元数 q [6:9] (w,x,y,z)
    constexpr int BG = 10;     // 陀螺偏差 bg [10:12]
    constexpr int BA = 13;     // 加速度偏差 ba [13:15]
}

/**
 * @brief 主状态结构体 (Nominal State)
 */
struct NominalState {
    Eigen::Vector3d position;        // 位置 [x, y, z]
    Eigen::Vector3d velocity;        // 速度 [vx, vy, vz] 
    Eigen::Quaterniond orientation;  // 姿态四元数 (body->world)
    Eigen::Vector3d gyro_bias;       // 陀螺仪偏差
    Eigen::Vector3d accel_bias;      // 加速度计偏差
    double timestamp;                // 时间戳
    
    NominalState() {
        position.setZero();
        velocity.setZero();
        orientation.setIdentity();
        gyro_bias.setZero();
        accel_bias.setZero();
        timestamp = 0.0;
    }
    
    /**
     * @brief 将主状态转换为向量形式
     */
    Eigen::VectorXd toVector() const {
        Eigen::VectorXd state(NOMINAL_STATE_SIZE);
        state.segment<3>(StateIndex::P) = position;
        state.segment<3>(StateIndex::V) = velocity;
        state.segment<4>(StateIndex::Q) = Eigen::Vector4d(orientation.w(), orientation.x(), orientation.y(), orientation.z());
        state.segment<3>(StateIndex::BG) = gyro_bias;
        state.segment<3>(StateIndex::BA) = accel_bias;
        return state;
    }
    
    /**
     * @brief 从向量形式恢复主状态
     */
    void fromVector(const Eigen::VectorXd& state_vec) {
        position = state_vec.segment<3>(StateIndex::P);
        velocity = state_vec.segment<3>(StateIndex::V);
        Eigen::Vector4d q_vec = state_vec.segment<4>(StateIndex::Q);
        orientation = Eigen::Quaterniond(q_vec[0], q_vec[1], q_vec[2], q_vec[3]);
        gyro_bias = state_vec.segment<3>(StateIndex::BG);
        accel_bias = state_vec.segment<3>(StateIndex::BA);
    }
};

/**
 * @brief IMU测量数据
 */
struct ImuData {
    Eigen::Vector3d angular_velocity;    // 角速度 [rad/s]
    Eigen::Vector3d linear_acceleration; // 线加速度 [m/s²]
    double timestamp;                    // 时间戳
    
    ImuData() {
        angular_velocity.setZero();
        linear_acceleration.setZero(); 
        timestamp = 0.0;
    }
};

/**
 * @brief DVL速度测量数据
 */
struct DvlData {
    Eigen::Vector3d velocity;    // 速度测量 [m/s]
    Eigen::Matrix3d covariance;  // 协方差矩阵
    double timestamp;            // 时间戳
    
    DvlData() {
        velocity.setZero();
        covariance.setIdentity();
        timestamp = 0.0;
    }
};

/**
 * @brief 深度传感器数据
 */
struct DepthData {
    double depth;               // 深度 [m] (正值表示水下深度)
    double variance;            // 测量方差
    double timestamp;           // 时间戳
    
    DepthData() {
        depth = 0.0;
        variance = 1.0;
        timestamp = 0.0;
    }
};

/**
 * @brief 航向角（偏航角）量测数据
 */
struct HeadingData {
    double yaw;                 // 航向角 [rad], 取值范围[-pi, pi]
    double variance;            // 测量方差 [rad^2]
    double timestamp;           // 时间戳
    
    HeadingData() {
        yaw = 0.0;
        variance = 0.1 * 0.1; // 合理默认方差
        timestamp = 0.0;
    }
};

/**
 * @brief 一程声学测距(OWTT)数据
 */
struct OwttData {
    std::string peer_ns;
    Eigen::Vector3d tx_position;
    Eigen::Matrix3d tx_position_covariance;
    Eigen::Matrix<double, STATE_SIZE, 3> tx_cross_cov_x_p;

    double range;
    double variance;
    double timestamp;

    OwttData() {
        peer_ns.clear();
        tx_position.setZero();
        tx_position_covariance.setIdentity();
        tx_cross_cov_x_p.setZero();
        range = 0.0;
        variance = 1.0;
        timestamp = 0.0;
    }
};

/**
 * @brief ESKF噪声参数
 */
struct NoiseParams {
    // IMU噪声参数
    double gyro_noise_std;           // 陀螺仪噪声标准差 [rad/s]  
    double accel_noise_std;          // 加速度计噪声标准差 [m/s²]
    double gyro_bias_std;            // 陀螺仪偏差随机游走标准差 [rad/s]
    double accel_bias_std;           // 加速度计偏差随机游走标准差 [m/s²]
    
    // 传感器测量噪声
    double dvl_noise_std;            // DVL速度测量噪声标准差 [m/s]
    double depth_noise_std;          // 深度测量噪声标准差 [m]
    
    NoiseParams() {
        gyro_noise_std = 0.01;       // 0.01 rad/s
        accel_noise_std = 0.1;       // 0.1 m/s²  
        gyro_bias_std = 1e-5;        // 1e-5 rad/s
        accel_bias_std = 1e-4;       // 1e-4 m/s²
        dvl_noise_std = 0.02;        // 0.02 m/s
        depth_noise_std = 0.01;      // 0.01 m
    }
};

/**
 * @brief 重力常数 (参考uuv_nav_fusion设定)
 */
constexpr double GRAVITY = 9.81;  // [m/s²] 与uuv_nav_fusion保持一致

/**
 * @brief ENU坐标系重力向量 (参考uuv_nav_fusion坐标系设定)
 * East-North-Up坐标系：Z轴朝上，重力向下为负
 */
const Eigen::Vector3d GRAVITY_ENU(0.0, 0.0, -GRAVITY);

/**
 * @brief 工具函数：轴角到斜对称矩阵
 */
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
    return skew;
}

/**
 * @brief 工具函数：四元数到旋转矩阵
 */
inline Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix();
}

/**
 * @brief 工具函数：小角度到旋转矩阵 (一阶近似)
 */
inline Eigen::Matrix3d smallAngleToRotationMatrix(const Eigen::Vector3d& angle) {
    return Eigen::Matrix3d::Identity() + skewSymmetric(angle);
}

} // namespace uuv_eskf_nav

#endif // UUV_ESKF_NAV_ESKF_TYPES_H
