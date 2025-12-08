#ifndef UUV_ESKF_NAV_SENSOR_MANAGER_H
#define UUV_ESKF_NAV_SENSOR_MANAGER_H

#include "eskf_types.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>

#include <memory>
#include <functional>

namespace uuv_eskf_nav {

/**
 * @brief 传感器数据管理器
 * 
 * 负责:
 * 1. 接收来自UUV仿真器的原始传感器数据
 * 2. 转换为ESKF算法需要的数据格式
 * 3. 提供回调函数接口供导航节点使用
 * 4. 管理传感器数据的时间同步和有效性检查
 */
class SensorManager {
public:
    // 传感器数据回调函数类型定义
    using ImuCallback = std::function<void(const ImuData&)>;
    using DvlCallback = std::function<void(const DvlData&)>;  
    using DepthCallback = std::function<void(const DepthData&)>;
    using HeadingCallback = std::function<void(const HeadingData&)>;
    // 地形匹配回调: x, y, var_x, var_y, timestamp
    using PositionCallback = std::function<void(double, double, double, double, double)>;

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param robot_name 机器人名称 (用于话题命名空间)
     */
    SensorManager(ros::NodeHandle& nh, const std::string& robot_name);

    /**
     * @brief 析构函数  
     */
    ~SensorManager() = default;

    /**
     * @brief 设置IMU数据回调函数
     * @param callback IMU数据回调函数
     */
    void setImuCallback(const ImuCallback& callback) {
        imu_callback_ = callback;
    }

    /**
     * @brief 设置DVL数据回调函数
     * @param callback DVL数据回调函数
     */
    void setDvlCallback(const DvlCallback& callback) {
        dvl_callback_ = callback;
    }

    /**
     * @brief 设置深度数据回调函数
     * @param callback 深度数据回调函数
     */
    void setDepthCallback(const DepthCallback& callback) {
        depth_callback_ = callback;
    }

    /**
     * @brief 设置航向角回调
     */
    void setHeadingCallback(const HeadingCallback& callback) {
        heading_callback_ = callback;
    }

    /**
     * @brief 设置地形匹配定位回调
     */
    void setTerrainPoseCallback(const PositionCallback& callback) {
        terrain_callback_ = callback;
    }

    /**
     * @brief 获取传感器数据统计信息
     */
    void printStatistics() const;

    /**
     * @brief 检查所有传感器是否正常接收数据
     * @param timeout_sec 超时时间(秒)
     * @return true if all sensors are active
     */
    bool areAllSensorsActive(double timeout_sec = 2.0) const;

private:
    /**
     * @brief IMU原始数据回调函数
     * @param msg IMU消息
     */
    void imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief DVL原始数据回调函数
     * @param msg DVL消息
     */
    void dvlRawCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg);

    /**
     * @brief 压力传感器原始数据回调函数
     * @param msg 压力传感器消息
     */
    void pressureRawCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);

    /**
     * @brief 地形匹配定位回调
     */
    void terrainPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
     * @brief 使用IMU四元数近似为航向量测（可选）
     */
    void headingFromImu(const sensor_msgs::Imu::ConstPtr& msg);

    // 注意：现在直接处理原始传感器数据，不再使用预处理回调函数

    /**
     * @brief 将压力转换为深度
     * @param pressure_kpa 压力值 (kPa)
     * @return 深度值 (m, 正值表示水下深度)
     */
    double pressureToDepth(double pressure_kpa) const;

    /**
     * @brief 检查IMU数据有效性
     * @param imu_data IMU数据
     * @return true if valid
     */
    bool validateImuData(const ImuData& imu_data) const;

    /**
     * @brief 检查DVL数据有效性
     * @param dvl_data DVL数据  
     * @return true if valid
     */
    bool validateDvlData(const DvlData& dvl_data) const;

    /**
     * @brief 检查深度数据有效性
     * @param depth_data 深度数据
     * @return true if valid
     */
    bool validateDepthData(const DepthData& depth_data) const;

    // ROS相关
    ros::NodeHandle& nh_;
    std::string robot_name_;

    // 订阅器
    ros::Subscriber imu_sub_;
    ros::Subscriber dvl_sub_;  
    ros::Subscriber pressure_sub_;
    ros::Subscriber terrain_pose_sub_;

    // 回调函数
    ImuCallback imu_callback_;
    DvlCallback dvl_callback_;
    DepthCallback depth_callback_;
    HeadingCallback heading_callback_;
    PositionCallback terrain_callback_;

    // 传感器数据统计
    struct SensorStats {
        size_t message_count;
        double last_message_time;
        double first_message_time;
        
        SensorStats() : message_count(0), last_message_time(0.0), first_message_time(0.0) {}
        
        void updateStats(double timestamp) {
            if (message_count == 0) {
                first_message_time = timestamp;
            }
            last_message_time = timestamp;
            message_count++;
        }
        
        double getFrequency() const {
            if (message_count < 2) return 0.0;
            double duration = last_message_time - first_message_time;
            return duration > 0 ? (message_count - 1) / duration : 0.0;
        }
    };
    
    mutable SensorStats imu_stats_;
    mutable SensorStats dvl_stats_;
    mutable SensorStats depth_stats_;

    // 最近一次IMU角速度缓存（用于DVL杠杆臂补偿）
    Eigen::Vector3d latest_imu_omega_ = Eigen::Vector3d::Zero();
    ros::Time latest_imu_time_ = ros::Time(0);

    // 物理常数 (与原uuv_nav_fusion保持一致)
    static constexpr double GRAVITY = 9.81;              // 重力加速度 [m/s²]
    static constexpr double WATER_DENSITY = 1025.0;      // 海水密度 [kg/m³]
    static constexpr double ATM_PRESSURE_KPA = 101.325;   // 标准大气压 [kPa]
    static constexpr double KPA_PER_METER = 9.80638;     // 深度转换系数 [kPa/m]

    // 数据有效性检查阈值
    static constexpr double MAX_GYRO_RATE = 10.0;        // 最大角速度 [rad/s]
    static constexpr double MAX_ACCEL_MAGNITUDE = 50.0;  // 最大加速度 [m/s²] 
    static constexpr double MAX_VELOCITY = 10.0;         // 最大速度 [m/s]
    static constexpr double MAX_DEPTH = 1000.0;          // 最大深度 [m]
    static constexpr double MIN_DEPTH = -10.0;           // 最小深度 [m] (允许一定的负深度)

    // 噪声模型参数
    static constexpr double DEFAULT_DVL_NOISE_STD = 0.02;    // DVL默认噪声标准差 [m/s]
    static constexpr double DEFAULT_DEPTH_NOISE_STD = 0.01;  // 深度默认噪声标准差 [m]
};

} // namespace uuv_eskf_nav

#endif // UUV_ESKF_NAV_SENSOR_MANAGER_H
