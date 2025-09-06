#include "uuv_eskf_nav/sensor_manager.h"
#include <iostream>
#include <cmath>

namespace uuv_eskf_nav {

SensorManager::SensorManager(ros::NodeHandle& nh, const std::string& robot_name)
    : nh_(nh), robot_name_(robot_name) {
    
    // 验证机器人名称
    if (robot_name_.empty()) {
        ROS_ERROR("SensorManager: 机器人名称不能为空!");
        return;
    }
    
    ROS_INFO("传感器管理器初始化 - 机器人: %s", robot_name_.c_str());
    
    // ⚠️ 重要修复：DVL坐标系转换问题
    // sensor_processor.py发布的DVL数据仍在dvl_link坐标系中，需要在ESKF中进行转换
    // 或者直接订阅原始DVL数据进行完整处理
    std::string imu_topic = "/" + robot_name_ + "/imu";  // 机器人IMU数据 (需要坐标变换)
    
    // 选择订阅原始DVL数据以获得完整控制权
    std::string dvl_topic = "/" + robot_name_ + "/dvl";           // 原始DVL数据
    std::string pressure_topic = "/" + robot_name_ + "/pressure"; // 原始压力传感器数据
    
    imu_sub_ = nh_.subscribe(imu_topic, 100, &SensorManager::imuRawCallback, this);
    dvl_sub_ = nh_.subscribe(dvl_topic, 100, &SensorManager::dvlRawCallback, this);
    pressure_sub_ = nh_.subscribe(pressure_topic, 100, &SensorManager::pressureRawCallback, this);
    
    ROS_INFO("传感器话题订阅成功:");
    ROS_INFO("  IMU: %s", imu_topic.c_str());
    ROS_INFO("  DVL: %s", dvl_topic.c_str());  
    ROS_INFO("  压力传感器: %s", pressure_topic.c_str());
}

void SensorManager::imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!imu_callback_) return;
    
    // 转换ROS消息为ESKF数据格式
    ImuData imu_data;
    imu_data.angular_velocity = Eigen::Vector3d(
        msg->angular_velocity.x,
        msg->angular_velocity.y, 
        msg->angular_velocity.z
    );
    imu_data.linear_acceleration = Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    imu_data.timestamp = msg->header.stamp.toSec();
    
    // 数据有效性检查
    if (!validateImuData(imu_data)) {
        ROS_WARN_THROTTLE(1.0, "SensorManager: IMU数据无效，跳过处理");
        return;
    }
    
    // 更新统计信息
    imu_stats_.updateStats(imu_data.timestamp);
    
    // 调用回调函数
    imu_callback_(imu_data);
}

void SensorManager::dvlRawCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
    if (!dvl_callback_) return;
    
    ROS_DEBUG("收到原始DVL数据 (dvl_link坐标系)");
    
    // 🎯 关键修复：DVL坐标系到base_link坐标系的转换
    // 根据eca_a9_sensors.xacro: <origin rpy="0 ${0.5*pi} 0" />
    // DVL传感器相对于base_link有90度俯仰角旋转
    
    // DVL坐标系中的速度
    Eigen::Vector3d dvl_velocity(
        msg->velocity.x,
        msg->velocity.y,
        msg->velocity.z
    );
    
    // DVL坐标系转换 (必须进行，因为ESKF需要base_link坐标系数据)
    // rpy="0 ${0.5*pi} 0" 表示绕Y轴旋转90度
    Eigen::Matrix3d R_dvl_to_base;
    R_dvl_to_base << 0, 0,  1,   // DVL_X ->  base_Z
                     0, 1,  0,   // DVL_Y ->  base_Y  
                    -1, 0,  0;   // DVL_Z -> -base_X
    
    // 转换到base_link坐标系
    Eigen::Vector3d base_velocity = R_dvl_to_base * dvl_velocity;
    
    DvlData dvl_data;
    dvl_data.velocity = base_velocity;
    
    // 简化协方差处理
    Eigen::Matrix3d dvl_covariance = Eigen::Matrix3d::Identity() * 0.01;
    dvl_data.covariance = R_dvl_to_base * dvl_covariance * R_dvl_to_base.transpose();
    
    dvl_data.timestamp = msg->header.stamp.toSec();
    
    // 数据有效性检查
    if (!validateDvlData(dvl_data)) {
        ROS_WARN_THROTTLE(1.0, "SensorManager: 转换后的DVL数据无效，跳过处理");
        return;
    }
    
    // 更新统计信息
    dvl_stats_.updateStats(dvl_data.timestamp);
    
    ROS_DEBUG("DVL速度 (base_link): [%.3f, %.3f, %.3f] m/s (原始: [%.3f, %.3f, %.3f])", 
             base_velocity.x(), base_velocity.y(), base_velocity.z(),
             dvl_velocity.x(), dvl_velocity.y(), dvl_velocity.z());
    
    // 调用回调函数
    dvl_callback_(dvl_data);
}

void SensorManager::pressureRawCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
    if (!depth_callback_) return;
    
    // 转换压力为深度
    double pressure_kpa = msg->fluid_pressure;
    double depth_m = pressureToDepth(pressure_kpa);
    
    // 转换为ESKF数据格式
    DepthData depth_data;
    depth_data.depth = depth_m;
    depth_data.variance = DEFAULT_DEPTH_NOISE_STD * DEFAULT_DEPTH_NOISE_STD;
    depth_data.timestamp = msg->header.stamp.toSec();
    
    // 根据深度调整不确定性 (深度越大，压力传感器可能越不准确)
    double depth_factor = 1.0 + 0.001 * std::abs(depth_m); // 每米深度增加0.1%的不确定性
    depth_data.variance *= depth_factor * depth_factor;
    
    // 数据有效性检查
    if (!validateDepthData(depth_data)) {
        ROS_WARN_THROTTLE(1.0, "SensorManager: 深度数据无效，跳过处理");
        return;
    }
    
    // 更新统计信息
    depth_stats_.updateStats(depth_data.timestamp);
    
    // 调用回调函数
    depth_callback_(depth_data);
}

double SensorManager::pressureToDepth(double pressure_kpa) const {
    // 使用与原uuv_nav_fusion相同的转换公式
    if (pressure_kpa >= ATM_PRESSURE_KPA) {
        return (pressure_kpa - ATM_PRESSURE_KPA) / KPA_PER_METER;
    } else {
        return 0.0; // 压力低于大气压时，深度为0
    }
}

bool SensorManager::validateImuData(const ImuData& imu_data) const {
    // 检查角速度幅值
    if (imu_data.angular_velocity.norm() > MAX_GYRO_RATE) {
        ROS_WARN_THROTTLE(1.0, "IMU角速度过大: %.3f rad/s", imu_data.angular_velocity.norm());
        return false;
    }
    
    // 检查加速度幅值
    if (imu_data.linear_acceleration.norm() > MAX_ACCEL_MAGNITUDE) {
        ROS_WARN_THROTTLE(1.0, "IMU加速度过大: %.3f m/s²", imu_data.linear_acceleration.norm());
        return false;  
    }
    
    // 检查数据是否为NaN
    if (!imu_data.angular_velocity.allFinite() || !imu_data.linear_acceleration.allFinite()) {
        ROS_WARN_THROTTLE(1.0, "IMU数据包含NaN或Inf");
        return false;
    }
    
    // 检查时间戳
    if (imu_data.timestamp <= 0) {
        ROS_WARN_THROTTLE(1.0, "IMU时间戳无效: %.3f", imu_data.timestamp);
        return false;
    }
    
    return true;
}

bool SensorManager::validateDvlData(const DvlData& dvl_data) const {
    // 检查速度幅值
    if (dvl_data.velocity.norm() > MAX_VELOCITY) {
        ROS_WARN_THROTTLE(1.0, "DVL速度过大: %.3f m/s", dvl_data.velocity.norm());
        return false;
    }
    
    // 检查数据是否为NaN
    if (!dvl_data.velocity.allFinite()) {
        ROS_WARN_THROTTLE(1.0, "DVL数据包含NaN或Inf");
        return false;
    }
    
    // 检查协方差矩阵是否正定
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(dvl_data.covariance);
    if (solver.eigenvalues().minCoeff() <= 0) {
        ROS_WARN_THROTTLE(1.0, "DVL协方差矩阵不是正定的");
        return false;
    }
    
    // 检查时间戳
    if (dvl_data.timestamp <= 0) {
        ROS_WARN_THROTTLE(1.0, "DVL时间戳无效: %.3f", dvl_data.timestamp);
        return false;
    }
    
    return true;
}

bool SensorManager::validateDepthData(const DepthData& depth_data) const {
    // 检查深度范围
    if (depth_data.depth < MIN_DEPTH || depth_data.depth > MAX_DEPTH) {
        ROS_WARN_THROTTLE(1.0, "深度超出合理范围: %.3f m", depth_data.depth);
        return false;
    }
    
    // 检查方差是否为正数
    if (depth_data.variance <= 0) {
        ROS_WARN_THROTTLE(1.0, "深度方差无效: %.6f", depth_data.variance);
        return false;
    }
    
    // 检查数据是否为NaN
    if (!std::isfinite(depth_data.depth) || !std::isfinite(depth_data.variance)) {
        ROS_WARN_THROTTLE(1.0, "深度数据包含NaN或Inf");
        return false;
    }
    
    // 检查时间戳
    if (depth_data.timestamp <= 0) {
        ROS_WARN_THROTTLE(1.0, "深度时间戳无效: %.3f", depth_data.timestamp);
        return false;
    }
    
    return true;
}

void SensorManager::printStatistics() const {
    ROS_INFO("=== 传感器数据统计 ===");
    ROS_INFO("IMU:");
    ROS_INFO("  消息计数: %zu", imu_stats_.message_count);
    ROS_INFO("  频率: %.1f Hz", imu_stats_.getFrequency());
    ROS_INFO("  最后消息时间: %.3f", imu_stats_.last_message_time);
    
    ROS_INFO("DVL:");
    ROS_INFO("  消息计数: %zu", dvl_stats_.message_count);
    ROS_INFO("  频率: %.1f Hz", dvl_stats_.getFrequency());
    ROS_INFO("  最后消息时间: %.3f", dvl_stats_.last_message_time);
    
    ROS_INFO("深度传感器:");
    ROS_INFO("  消息计数: %zu", depth_stats_.message_count);
    ROS_INFO("  频率: %.1f Hz", depth_stats_.getFrequency());
    ROS_INFO("  最后消息时间: %.3f", depth_stats_.last_message_time);
}

// 注意：不再使用预处理数据，直接处理原始传感器数据以获得完整控制权

bool SensorManager::areAllSensorsActive(double timeout_sec) const {
    double current_time = ros::Time::now().toSec();
    
    bool imu_active = (imu_stats_.message_count > 0) && 
                     (current_time - imu_stats_.last_message_time < timeout_sec);
    bool dvl_active = (dvl_stats_.message_count > 0) && 
                     (current_time - dvl_stats_.last_message_time < timeout_sec);
    bool depth_active = (depth_stats_.message_count > 0) && 
                       (current_time - depth_stats_.last_message_time < timeout_sec);
    
    if (!imu_active) {
        ROS_WARN_THROTTLE(5.0, "IMU传感器数据超时 (%.1fs)", 
                         current_time - imu_stats_.last_message_time);
    }
    if (!dvl_active) {
        ROS_WARN_THROTTLE(5.0, "DVL传感器数据超时 (%.1fs)", 
                         current_time - dvl_stats_.last_message_time);
    }
    if (!depth_active) {
        ROS_WARN_THROTTLE(5.0, "深度传感器数据超时 (%.1fs)", 
                         current_time - depth_stats_.last_message_time);
    }
    
    return imu_active && dvl_active && depth_active;
}

} // namespace uuv_eskf_nav
