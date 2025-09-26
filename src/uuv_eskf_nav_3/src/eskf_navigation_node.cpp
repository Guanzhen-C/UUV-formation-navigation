#include "uuv_eskf_nav/eskf_core.h"
#include "uuv_eskf_nav/sensor_manager.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT.h>
#include <uuv_sensor_ros_plugins_msgs/Method3SenderState.h>
#include <uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped.h>

#include <memory>
#include <string>

namespace uuv_eskf_nav {

/**
 * @brief ESKF导航节点类
 * 
 * 功能:
 * 1. 集成ESKF算法和传感器管理
 * 2. 发布导航结果 (nav_msgs/Odometry)
 * 3. 发布TF变换
 * 4. 提供导航状态监控
 */
class EskfNavigationNode {
public:
    EskfNavigationNode() : nh_("~") {
        // 读取参数
        if (!loadParameters()) {
            ROS_FATAL("参数加载失败!");
            ros::shutdown();
            return;
        }
        
        // 初始化改进ESKF算法
        bool imu_accel_includes_gravity = false;
        nh_.param<bool>("imu/accel_includes_gravity", imu_accel_includes_gravity, false);
        eskf_ = std::make_unique<EskfCore>(noise_params_, imu_accel_includes_gravity);
        
        // 设置地球自转补偿参数
        bool enable_earth_rotation = false;
        double mission_latitude_deg = 18.25;  // 三亚纬度
        nh_.param<bool>("algorithm_params/enable_earth_rotation", enable_earth_rotation, false);
        nh_.param<double>("algorithm_params/mission_latitude_deg", mission_latitude_deg, 30.0);
        
        double mission_latitude_rad = mission_latitude_deg * M_PI / 180.0;
        eskf_->setEarthRotationParams(enable_earth_rotation, mission_latitude_rad);
        
        ROS_INFO("地球自转补偿: %s, 任务纬度: %.1f°", 
                 enable_earth_rotation ? "启用" : "关闭", mission_latitude_deg);
        
        // 初始化传感器管理器
        sensor_manager_ = std::make_unique<SensorManager>(nh_, robot_name_);
        
        // 设置传感器数据回调
        sensor_manager_->setImuCallback(
            std::bind(&EskfNavigationNode::onImuData, this, std::placeholders::_1));
        if (enable_dvl_) {
            sensor_manager_->setDvlCallback(
                std::bind(&EskfNavigationNode::onDvlData, this, std::placeholders::_1));
        }
        sensor_manager_->setDepthCallback(
            std::bind(&EskfNavigationNode::onDepthData, this, std::placeholders::_1));
        if (enable_heading_) {
            sensor_manager_->setHeadingCallback(
                std::bind(&EskfNavigationNode::onHeadingData, this, std::placeholders::_1));
        }
        
        // 初始化发布器
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/eskf/odometry/filtered", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/eskf/pose", 10);
        method3_pub_ = nh_.advertise<uuv_sensor_ros_plugins_msgs::Method3SenderState>("/" + robot_name_ + "/rpt/method3_sender_state", 10);
        
        // 初始化地面真值订阅器 (用于动态初始化)
        std::string gt_topic = "/" + robot_name_ + "/pose_gt";
        gt_pose_sub_ = nh_.subscribe(gt_topic, 10, &EskfNavigationNode::gtPoseCallback, this);

        // 订阅OWTT单程测距
        std::string owtt_topic = "/" + robot_name_ + "/rpt/range_owtt";
        owtt_sub_ = nh_.subscribe(owtt_topic, 10, &EskfNavigationNode::owttCallback, this);
        
        // 初始化TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
        
        // 初始化定时器
        status_timer_ = nh_.createTimer(ros::Duration(1.0), 
                                       &EskfNavigationNode::statusTimerCallback, this);
        
        ROS_INFO("ESKF导航节点初始化完成");
        ROS_INFO("机器人名称: %s", robot_name_.c_str());
        ROS_INFO("等待传感器数据...");
        
        // 延迟初始化ESKF (等待传感器数据)
        initialization_timer_ = nh_.createTimer(ros::Duration(0.1),
                                               &EskfNavigationNode::initializationTimerCallback, this);
    }
    
    ~EskfNavigationNode() = default;

private:
    bool loadParameters() {
        // 机器人名称
        if (!nh_.getParam("robot_name", robot_name_)) {
            ROS_ERROR("必须在配置文件中定义robot_name参数!");
            return false;
        }
        if (robot_name_.empty()) {
            ROS_ERROR("robot_name参数不能为空!");
            return false;
        }
        
        // 坐标系参数
        nh_.param<std::string>("world_frame", world_frame_, "odom");
        nh_.param<std::string>("base_link_frame", base_link_frame_, robot_name_ + "/base_link");
        
        // ESKF噪声参数
        nh_.param<double>("imu/gyro_noise_std", noise_params_.gyro_noise_std, 0.01);
        nh_.param<double>("imu/accel_noise_std", noise_params_.accel_noise_std, 0.1);
        nh_.param<double>("imu/gyro_bias_std", noise_params_.gyro_bias_std, 1e-5);
        nh_.param<double>("imu/accel_bias_std", noise_params_.accel_bias_std, 1e-4);
        nh_.param<double>("dvl/noise_std", noise_params_.dvl_noise_std, 0.02);
        nh_.param<double>("depth/noise_std", noise_params_.depth_noise_std, 0.01);
        // 航向噪声
        nh_.param<double>("heading/noise_std", heading_noise_std_, 0.1);
        
        // 初始状态参数
        std::vector<double> init_pos, init_vel, init_att;
        nh_.param<std::vector<double>>("initial_state/position", init_pos, {0.0, 0.0, 0.0});
        nh_.param<std::vector<double>>("initial_state/velocity", init_vel, {0.0, 0.0, 0.0});
        nh_.param<std::vector<double>>("initial_state/attitude", init_att, {0.0, 0.0, 0.0});
        
        if (init_pos.size() != 3 || init_vel.size() != 3 || init_att.size() != 3) {
            ROS_ERROR("初始状态参数维度错误!");
            return false;
        }
        
        initial_state_.position = Eigen::Vector3d(init_pos[0], init_pos[1], init_pos[2]);
        initial_state_.velocity = Eigen::Vector3d(init_vel[0], init_vel[1], init_vel[2]);
        
        // 欧拉角转四元数 (roll, pitch, yaw)
        tf2::Quaternion q;
        q.setRPY(init_att[0], init_att[1], init_att[2]);
        initial_state_.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
        
        initial_state_.gyro_bias.setZero();
        initial_state_.accel_bias.setZero();
        initial_state_.timestamp = 0.0;
        
        // 初始协方差
        std::vector<double> init_cov_diag;
        nh_.param<std::vector<double>>("initial_covariance_diagonal", init_cov_diag, 
            {1.0, 1.0, 0.1,    // 位置标准差 [m]
             0.1, 0.1, 0.1,    // 速度标准差 [m/s]  
             0.1, 0.1, 0.2,    // 姿态标准差 [rad]
             0.01, 0.01, 0.01, // 陀螺偏差标准差 [rad/s]
             0.1, 0.1, 0.1});  // 加速度偏差标准差 [m/s²]
        
        if (init_cov_diag.size() != STATE_SIZE) {
            ROS_ERROR("初始协方差对角元素个数错误! 期望%d个，实际%zu个", STATE_SIZE, init_cov_diag.size());
            return false;
        }
        
        initial_covariance_ = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
        for (int i = 0; i < STATE_SIZE; ++i) {
            initial_covariance_(i, i) = init_cov_diag[i] * init_cov_diag[i]; // 方差 = 标准差²
        }
        
        // 其他参数
        nh_.param<bool>("publish_tf", publish_tf_, true);
        nh_.param<double>("max_initialization_time", max_init_time_, 10.0);
        nh_.param<bool>("sensors/enable_dvl", enable_dvl_, true);
        nh_.param<bool>("sensors/enable_heading", enable_heading_, true);
        // 将heading噪声传入ESKF（通过回调构造量测时使用）
        heading_variance_ = heading_noise_std_ * heading_noise_std_;
        
        ROS_INFO("参数加载完成:");
        ROS_INFO("  噪声参数: gyro_std=%.4f, accel_std=%.4f, dvl_std=%.4f, depth_std=%.4f",
                noise_params_.gyro_noise_std, noise_params_.accel_noise_std,
                noise_params_.dvl_noise_std, noise_params_.depth_noise_std);
        ROS_INFO("  传感器开关: DVL=%s, Heading=%s", enable_dvl_?"ON":"OFF", enable_heading_?"ON":"OFF");
        
        return true;
    }
    
    void initializationTimerCallback(const ros::TimerEvent&) {
        static double init_start_time = ros::Time::now().toSec();
        double elapsed_time = ros::Time::now().toSec() - init_start_time;
        
        // 检查是否超时
        if (elapsed_time > max_init_time_) {
            ROS_ERROR("ESKF初始化超时 (%.1f秒)，请检查传感器数据!", elapsed_time);
            initialization_timer_.stop();
            return;
        }
        
        // 检查传感器是否都有数据 (放宽要求，只需要IMU数据即可先初始化)
        if (elapsed_time > 5.0 && !sensor_manager_->areAllSensorsActive(2.0)) {
            ROS_WARN("传感器数据不全，但已等待%.1f秒，继续初始化...", elapsed_time);
        }
        
        
        // 实时获取地面真值进行动态初始化
        nav_msgs::Odometry::ConstPtr gt_pose;
        try {
            ROS_INFO("正在获取地面真值进行动态初始化...");
            gt_pose = ros::topic::waitForMessage<nav_msgs::Odometry>("/" + robot_name_ + "/pose_gt", ros::Duration(3.0));
            if (!gt_pose) {
                ROS_WARN("无法获取地面真值，使用配置的初始状态");
                eskf_->initialize(initial_state_, initial_covariance_);
            } else {
                NominalState dynamic_initial_state = initial_state_;
                
                // 从地面真值获取初始位置
                dynamic_initial_state.position.x() = gt_pose->pose.pose.position.x;
                dynamic_initial_state.position.y() = gt_pose->pose.pose.position.y;
                dynamic_initial_state.position.z() = gt_pose->pose.pose.position.z;
        
                // 从地面真值获取初始速度
                dynamic_initial_state.velocity.x() = gt_pose->twist.twist.linear.x;
                dynamic_initial_state.velocity.y() = gt_pose->twist.twist.linear.y;
                dynamic_initial_state.velocity.z() = gt_pose->twist.twist.linear.z;
                
                // 从地面真值获取初始姿态
                dynamic_initial_state.orientation.w() = gt_pose->pose.pose.orientation.w;
                dynamic_initial_state.orientation.x() = gt_pose->pose.pose.orientation.x;
                dynamic_initial_state.orientation.y() = gt_pose->pose.pose.orientation.y;
                dynamic_initial_state.orientation.z() = gt_pose->pose.pose.orientation.z;
                
                dynamic_initial_state.timestamp = gt_pose->header.stamp.toSec();
        
                ROS_INFO("✅ 使用地面真值进行动态初始化:");
                ROS_INFO("  位置: [%.2f, %.2f, %.2f]", 
                         dynamic_initial_state.position.x(), 
                         dynamic_initial_state.position.y(), 
                         dynamic_initial_state.position.z());
                ROS_INFO("  速度: [%.2f, %.2f, %.2f]", 
                         dynamic_initial_state.velocity.x(), 
                         dynamic_initial_state.velocity.y(), 
                         dynamic_initial_state.velocity.z());
                
                // 初始化ESKF
                if (!eskf_->initialize(dynamic_initial_state, initial_covariance_)) {
                    ROS_ERROR("ESKF动态初始化失败!");
                    return;
                }
            }
        } catch (const std::exception& e) {
            ROS_WARN("获取地面真值失败: %s, 使用配置初始状态", e.what());
            if (!eskf_->initialize(initial_state_, initial_covariance_)) {
                ROS_ERROR("ESKF备用初始化失败!");
                return;
            }
        }
        
        // 停止初始化定时器
        initialization_timer_.stop();
        
        // 开始导航
        ROS_INFO("ESKF导航系统启动成功!");
        sensor_manager_->printStatistics();
        
        is_initialized_ = true;
    }
    
    void statusTimerCallback(const ros::TimerEvent&) {
        if (!is_initialized_) return;
        
        // 检查传感器状态
        if (!sensor_manager_->areAllSensorsActive()) {
            ROS_WARN_THROTTLE(5.0, "部分传感器数据丢失!");
        }
        
        // 打印导航状态
        if (eskf_->isInitialized()) {
            const NominalState& state = eskf_->getNominalState();
            const Eigen::MatrixXd& cov = eskf_->getCovariance();
            
            // 计算位置、速度、姿态的不确定性
            double pos_uncertainty = std::sqrt(cov.block<3,3>(0,0).trace() / 3.0);
            double vel_uncertainty = std::sqrt(cov.block<3,3>(3,3).trace() / 3.0);  
            double att_uncertainty = std::sqrt(cov.block<3,3>(6,6).trace() / 3.0) * 180.0 / M_PI;
            
            ROS_INFO_THROTTLE(10.0, "导航状态 - 位置: [%.2f, %.2f, %.2f] (±%.3fm), "
                             "速度: [%.2f, %.2f, %.2f] (±%.3fm/s), 姿态不确定性: ±%.2f°",
                             state.position.x(), state.position.y(), state.position.z(), pos_uncertainty,
                             state.velocity.x(), state.velocity.y(), state.velocity.z(), vel_uncertainty,
                             att_uncertainty);

            // 定期发布Method3 发送端状态（1Hz）
            publishMethod3SenderState(ros::Time::now().toSec());
        }
    }
    
    void gtPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_gt_pose_ = *msg;
        has_gt_pose_ = true;
    }
    
    void onImuData(const ImuData& imu_data) {
        if (!is_initialized_ || !eskf_->isInitialized()) {
            // 缓存第一个IMU数据用于后续的圆锥补偿
            if (!has_previous_imu_) {
                previous_imu_data_ = imu_data;
                has_previous_imu_ = true;
            }
            return;
        }
        
        // 改进ESKF预测 (需要当前和前一时刻IMU数据)
        if (has_previous_imu_) {
            if (!eskf_->predictWithImprovedMechanization(imu_data, previous_imu_data_)) {
                ROS_WARN_THROTTLE(1.0, "改进ESKF预测步骤失败");
                previous_imu_data_ = imu_data;  // 更新缓存
                return;
            }
        } else {
            // 第一次接收到数据，只缓存不处理
            has_previous_imu_ = true;
            previous_imu_data_ = imu_data;
            return;
        }
        
        // 更新IMU数据缓存
        previous_imu_data_ = imu_data;
        
        // 发布导航结果 (以IMU频率发布)
        publishNavigationResult(imu_data.timestamp);
    }

    // 航向噪声（标准差与方差）
    double heading_noise_std_ = 0.1;
    double heading_variance_ = 0.01;
    
    void onDvlData(const DvlData& dvl_data) {
        if (!is_initialized_ || !eskf_->isInitialized()) return;
        
        // DVL速度更新
        if (!eskf_->updateWithDvl(dvl_data)) {
            ROS_WARN_THROTTLE(1.0, "ESKF DVL更新失败");
            return;
        }
        
        ROS_DEBUG("DVL更新完成: 速度=[%.3f, %.3f, %.3f]", 
                 dvl_data.velocity.x(), dvl_data.velocity.y(), dvl_data.velocity.z());
    }
    
    void onDepthData(const DepthData& depth_data) {
        if (!is_initialized_ || !eskf_->isInitialized()) return;
        
        // 深度更新
        if (!eskf_->updateWithDepth(depth_data)) {
            ROS_WARN_THROTTLE(1.0, "ESKF深度更新失败");
            return;
        }
        
        ROS_DEBUG("深度更新完成: 深度=%.3f m", depth_data.depth);
    }

    void onHeadingData(const HeadingData& heading_data) {
        if (!is_initialized_ || !eskf_->isInitialized()) return;
        if (!eskf_->updateWithHeading(heading_data)) {
            ROS_WARN_THROTTLE(1.0, "ESKF航向更新失败");
            return;
        }
    }

    void owttCallback(const uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT::ConstPtr& msg) {
        if (!is_initialized_ || !eskf_->isInitialized()) return;

        // 构造OwttData
        OwttData owtt;
        owtt.peer_ns = msg->from_ns;
        owtt.range = msg->range;
        owtt.variance = msg->variance;
        owtt.timestamp = msg->t_rx.toSec();

        // 发送端位置与协方差（行优先）
        owtt.tx_position.x() = msg->tx_pos.pos.pos.x;
        owtt.tx_position.y() = msg->tx_pos.pos.pos.y;
        owtt.tx_position.z() = msg->tx_pos.pos.pos.z;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                owtt.tx_position_covariance(i, j) = msg->tx_pos.pos.covariance[i * 3 + j];
            }
        }
        if (msg->tx_cross_cov_x_p.size() == 45) {
            for (int r = 0; r < STATE_SIZE; ++r) {
                for (int c = 0; c < 3; ++c) {
                    owtt.tx_cross_cov_x_p(r, c) = msg->tx_cross_cov_x_p[r * 3 + c];
                }
            }
        } else {
            owtt.tx_cross_cov_x_p.setZero();
        }

        if (!eskf_->updateWithOwttRange(owtt)) {
            ROS_WARN_THROTTLE(1.0, "ESKF OWTT更新失败");
            return;
        }
    }
    
    void publishNavigationResult(double timestamp) {
        if (!eskf_->isInitialized()) return;
        
        const NominalState& state = eskf_->getNominalState();
        
        // 发布Odometry消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time(timestamp);
        odom_msg.header.frame_id = world_frame_;
        odom_msg.child_frame_id = base_link_frame_;
        
        // 位置
        odom_msg.pose.pose.position.x = state.position.x();
        odom_msg.pose.pose.position.y = state.position.y();
        odom_msg.pose.pose.position.z = state.position.z();
        
        // 姿态  
        odom_msg.pose.pose.orientation.w = state.orientation.w();
        odom_msg.pose.pose.orientation.x = state.orientation.x();
        odom_msg.pose.pose.orientation.y = state.orientation.y();
        odom_msg.pose.pose.orientation.z = state.orientation.z();
        
        // 速度
        odom_msg.twist.twist.linear.x = state.velocity.x();
        odom_msg.twist.twist.linear.y = state.velocity.y();
        odom_msg.twist.twist.linear.z = state.velocity.z();
        
        // 协方差 (只填充位置和姿态部分)
        const Eigen::MatrixXd& P = eskf_->getCovariance();
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i < 3 && j < 3) {
                    // 位置协方差
                    odom_msg.pose.covariance[i*6 + j] = P(i, j);
                } else if (i >= 3 && j >= 3) {
                    // 姿态协方差 (来自误差状态的姿态部分)
                    odom_msg.pose.covariance[i*6 + j] = P(6 + (i-3), 6 + (j-3));
                }
            }
        }
        
        // 速度协方差
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                odom_msg.twist.covariance[i*6 + j] = P(3 + i, 3 + j);
            }
        }
        
        odom_pub_.publish(odom_msg);
        
        // 发布PoseStamped消息
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose.pose;
        pose_pub_.publish(pose_msg);
        
        // 发布TF变换
        if (publish_tf_) {
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time(timestamp);
            transform.header.frame_id = world_frame_;
            transform.child_frame_id = base_link_frame_;
            
            transform.transform.translation.x = state.position.x();
            transform.transform.translation.y = state.position.y();
            transform.transform.translation.z = state.position.z();
            transform.transform.rotation = odom_msg.pose.pose.orientation;
            
            tf_broadcaster_->sendTransform(transform);
        }
    }

    void publishMethod3SenderState(double timestamp) {
        if (!eskf_ || !eskf_->isInitialized()) return;

        const NominalState& state = eskf_->getNominalState();
        const Eigen::MatrixXd& P = eskf_->getCovariance();

        uuv_sensor_ros_plugins_msgs::Method3SenderState m3;
        m3.header.stamp = ros::Time(timestamp);
        m3.header.frame_id = world_frame_;
        m3.ns = robot_name_;

        m3.pos.header = m3.header;
        m3.pos.pos.pos.x = state.position.x();
        m3.pos.pos.pos.y = state.position.y();
        m3.pos.pos.pos.z = state.position.z();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m3.pos.pos.covariance[i * 3 + j] = P(StateIndex::DP + i, StateIndex::DP + j);
            }
        }
        for (int r = 0; r < STATE_SIZE; ++r) {
            for (int c = 0; c < 3; ++c) {
                m3.cross_cov_x_p[r * 3 + c] = P(r, StateIndex::DP + c);
            }
        }

        method3_pub_.publish(m3);
    }
    
    // 私有成员变量
    ros::NodeHandle nh_;
    
    // 参数
    std::string robot_name_;
    std::string world_frame_;
    std::string base_link_frame_;
    bool publish_tf_;
    double max_init_time_;
    bool enable_dvl_ = true;
    bool enable_heading_ = true;
    
    // ESKF相关
    std::unique_ptr<EskfCore> eskf_;
    std::unique_ptr<SensorManager> sensor_manager_;
    NoiseParams noise_params_;
    NominalState initial_state_;
    Eigen::MatrixXd initial_covariance_;
    
    // IMU数据缓存 (用于改进机械编排的圆锥补偿)
    ImuData previous_imu_data_;
    bool has_previous_imu_ = false;
    
    // ROS发布器和广播器
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher method3_pub_;
    ros::Subscriber owtt_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 定时器
    ros::Timer status_timer_;
    ros::Timer initialization_timer_;
    
    // 地面真值订阅器和数据
    ros::Subscriber gt_pose_sub_;
    nav_msgs::Odometry current_gt_pose_;
    bool has_gt_pose_ = false;
    
    // 状态标志
    bool is_initialized_ = false;
};

} // namespace uuv_eskf_nav

int main(int argc, char** argv) {
    ros::init(argc, argv, "eskf_navigation_node");
    
    try {
        uuv_eskf_nav::EskfNavigationNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("ESKF导航节点异常退出: %s", e.what());
        return 1;
    }
    
    return 0;
}
