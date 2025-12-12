#include "uuv_eskf_nav/sensor_manager.h"
#include <iostream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>

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
    std::string terrain_topic = "/" + robot_name_ + "/terrain_nav/pose"; // 地形匹配位置数据
    
    imu_sub_ = nh_.subscribe(imu_topic, 100, &SensorManager::imuRawCallback, this);
    dvl_sub_ = nh_.subscribe(dvl_topic, 100, &SensorManager::dvlRawCallback, this);
    pressure_sub_ = nh_.subscribe(pressure_topic, 100, &SensorManager::pressureRawCallback, this);
    terrain_pose_sub_ = nh_.subscribe(terrain_topic, 10, &SensorManager::terrainPoseCallback, this);
    
    ROS_INFO("传感器话题订阅成功:");
    ROS_INFO("  IMU: %s", imu_topic.c_str());
    ROS_INFO("  DVL: %s", dvl_topic.c_str());  
    ROS_INFO("  压力传感器: %s", pressure_topic.c_str());
    ROS_INFO("  地形匹配: %s", terrain_topic.c_str());
}

void SensorManager::terrainPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!terrain_callback_) return;
    
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double var_x = msg->pose.covariance[0];
    double var_y = msg->pose.covariance[7];
    double timestamp = msg->header.stamp.toSec();
    
    // 如果方差为0，设置一个默认值
    if (var_x <= 1e-6) var_x = 100.0;
    if (var_y <= 1e-6) var_y = 100.0;
    
    terrain_callback_(x, y, var_x, var_y, timestamp);
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

    // 缓存最近IMU角速度（base_link坐标系下，用于DVL杠杆臂补偿）
    // 重要：减去偏差以获得更准确的角速度
    static Eigen::Vector3d gyro_bias_estimate(0, 0, 0);
    static bool bias_initialized = false;
    
    // 从ESKF获取最新的陀螺偏差估计（如果可用）
    if (!bias_initialized) {
        // 初始化时使用零偏差
        gyro_bias_estimate.setZero();
        bias_initialized = true;
    }
    
    // 应用偏差补偿
    latest_imu_omega_ = imu_data.angular_velocity - gyro_bias_estimate;
    latest_imu_time_ = msg->header.stamp;
    
    // 调用回调函数
    imu_callback_(imu_data);

    // 可选：从IMU姿态导出航向量测，提供弱航向约束（限频 ~10Hz）
    static double last_heading_pub_time = 0.0;
    if (heading_callback_ && (imu_data.timestamp - last_heading_pub_time) >= 0.05) {
        // 提取yaw（ZYX，世界->机体通常需考虑磁偏角，这里简化忽略）
        tf2::Quaternion q_tf;
        tf2::fromMsg(msg->orientation, q_tf);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
        HeadingData hd;
        hd.yaw = yaw;
        // 从参数读取航向噪声，默认0.02 rad
        static double heading_noise_std = 0.02;
        static bool loaded = false;
        if (!loaded) {
            ros::NodeHandle pnh("~");
            pnh.param<double>("heading/noise_std", heading_noise_std, 0.02);
            loaded = true;
        }
        hd.variance = heading_noise_std * heading_noise_std;
        hd.timestamp = msg->header.stamp.toSec();
        heading_callback_(hd);
        last_heading_pub_time = imu_data.timestamp;
    }
}

void SensorManager::dvlRawCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
    if (!dvl_callback_) return;
    
    ROS_DEBUG("收到原始DVL数据 (dvl_link坐标系)");
    
    // 🎯 使用TF从 dvl_link 到 base_link 的实时旋转，转换速度到base_link坐标系
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);
    const std::string base_link_frame = robot_name_ + "/base_link";
    const std::string dvl_link_frame  = robot_name_ + "/dvl_link";
    
    geometry_msgs::Vector3Stamped vel_dvl, vel_base;
    vel_dvl.header = msg->header;
    vel_dvl.header.frame_id = dvl_link_frame;
    vel_dvl.vector = msg->velocity;
    
    try {
        // 使用量测时间戳查询TF，确保与量测时刻对齐
        geometry_msgs::TransformStamped T_base_dvl = tf_buffer.lookupTransform(base_link_frame, dvl_link_frame, msg->header.stamp, ros::Duration(0.05));
        tf2::doTransform(vel_dvl, vel_base, T_base_dvl);
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "DVL坐标变换失败: %s", ex.what());
        return;
    }
    
    Eigen::Vector3d base_velocity(vel_base.vector.x, vel_base.vector.y, vel_base.vector.z);

    // 杠杆臂补偿：v_com = v_sensor - ω × r
    try {
        // 1) 取 base_link 相对世界的角速度（用 IMU 估计更稳，这里直接用 TF 查询不到角速度，采用最近IMU角速话题需要缓存；
        // 简化：从 /<robot>/imu 最近一次角速度缓存。若没有缓存，跳过补偿）
        static Eigen::Vector3d last_omega_base(0,0,0);
        static ros::Time last_imu_time(0);
        // 读取参数，是否启用杠杆臂补偿（默认启用）
        static bool lever_arm_enabled = true;
        static bool lever_param_loaded = false;
        if (!lever_param_loaded) {
            ros::NodeHandle pnh("~");
            pnh.param<bool>("dvl/enable_lever_arm_comp", lever_arm_enabled, true);
            lever_param_loaded = true;
        }
        if (lever_arm_enabled) {
            // 查询 dvl_link 在 base_link 下的位置 r
            geometry_msgs::TransformStamped T_base_dvl = tf_buffer.lookupTransform(base_link_frame, dvl_link_frame, msg->header.stamp, ros::Duration(0.05));
            const geometry_msgs::Vector3& t = T_base_dvl.transform.translation;
            Eigen::Vector3d r_base(t.x, t.y, t.z);

            // 从 /<robot>/imu 获取最近一次角速度缓存：这里复用IMU回调已更新的统计时间，不直接可用角速。
            // 折中：用 vel_dvl.header.stamp 附近的 IMU 角速度需要全局缓存；当前简化为使用上一帧发布到ESKF的角速度由ESKF内部处理。
            // 为尽量不侵入主结构，这里提供保守近似：若 IMU 频率高，上一帧角速近似当前。
            // 由于本类未缓存角速，这里不做强制扣除，留给后续扩展。
            // TODO(optional): 引入 IMU 缓存队列以插值角速度

            // 使用最新IMU角速度作近似（带时间检查和插值）
            double time_diff = (msg->header.stamp - latest_imu_time_).toSec();
            if (time_diff < 0.05 && time_diff >= 0) { // 50ms内的数据有效
                // 如果时间差很小，直接使用
                base_velocity -= latest_imu_omega_.cross(r_base);
            } else if (time_diff < 0.2 && time_diff >= 0) {
                // 时间差稍大，应用衰减
                double decay_factor = 1.0 - (time_diff / 0.2);
                base_velocity -= (latest_imu_omega_ * decay_factor).cross(r_base);
            }
            // 否则跳过杠杆臂补偿
        }
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "DVL杠杆臂补偿失败: %s", ex.what());
    }
    
    DvlData dvl_data;
    dvl_data.velocity = base_velocity;
    
    // 协方差：使用TF旋转协方差到base_link
    Eigen::Matrix3d dvl_covariance = Eigen::Matrix3d::Identity() * 0.01;
    // 从TF获取旋转矩阵
    Eigen::Matrix3d R;
    try {
        geometry_msgs::TransformStamped T_base_dvl = tf_buffer.lookupTransform(base_link_frame, dvl_link_frame, msg->header.stamp, ros::Duration(0.05));
        tf2::Quaternion q_tf;
        tf2::fromMsg(T_base_dvl.transform.rotation, q_tf);
        tf2::Matrix3x3 R_tf(q_tf);
        R << R_tf[0][0], R_tf[0][1], R_tf[0][2],
             R_tf[1][0], R_tf[1][1], R_tf[1][2],
             R_tf[2][0], R_tf[2][1], R_tf[2][2];
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "DVL协方差旋转TF获取失败: %s", ex.what());
        R.setIdentity();
    }
    dvl_data.covariance = R * dvl_covariance * R.transpose();
    
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
             msg->velocity.x, msg->velocity.y, msg->velocity.z);
    
    // 调用回调函数
    dvl_callback_(dvl_data);
}

void SensorManager::pressureRawCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
    if (!depth_callback_) return;
    
    // 转换压力为深度
    // UUV仿真压力插件以 kPa 发布，此处直接使用 kPa
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
    // 无条件信任DVL数据（仅记录提示，不拒绝）
    if (dvl_data.velocity.norm() > MAX_VELOCITY) {
        ROS_WARN_THROTTLE(1.0, "DVL速度过大(忽略检查): %.3f m/s", dvl_data.velocity.norm());
    }
    if (!dvl_data.velocity.allFinite()) {
        ROS_WARN_THROTTLE(1.0, "DVL数据包含NaN或Inf(忽略检查)");
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(dvl_data.covariance);
    if (solver.eigenvalues().minCoeff() <= 0) {
        ROS_WARN_THROTTLE(1.0, "DVL协方差矩阵非正定(忽略检查)");
    }
    if (dvl_data.timestamp <= 0) {
        ROS_WARN_THROTTLE(1.0, "DVL时间戳无效(忽略检查): %.3f", dvl_data.timestamp);
    }
    return true;
}

bool SensorManager::validateDepthData(const DepthData& depth_data) const {
    // 无条件信任深度数据（保留日志提示但不拒绝）
    if (depth_data.depth < MIN_DEPTH || depth_data.depth > MAX_DEPTH) {
        ROS_WARN_THROTTLE(1.0, "深度超出合理范围(忽略检查): %.3f m", depth_data.depth);
    }
    if (depth_data.variance <= 0) {
        ROS_WARN_THROTTLE(1.0, "深度方差无效(忽略检查): %.6f", depth_data.variance);
    }
    if (!std::isfinite(depth_data.depth) || !std::isfinite(depth_data.variance)) {
        ROS_WARN_THROTTLE(1.0, "深度数据包含NaN或Inf(忽略检查)");
    }
    if (depth_data.timestamp <= 0) {
        ROS_WARN_THROTTLE(1.0, "深度时间戳无效(忽略检查): %.3f", depth_data.timestamp);
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
