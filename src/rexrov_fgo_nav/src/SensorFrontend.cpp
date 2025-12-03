#include "rexrov_fgo_nav/SensorFrontend.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace rexrov_fgo_nav {

SensorFrontend::SensorFrontend(ros::NodeHandle& nh, const std::string& robot_name)
    : nh_(nh), robot_name_(robot_name), tf_listener_(tf_buffer_)
{
    std::string imu_topic = "/" + robot_name_ + "/imu";
    std::string dvl_topic = "/" + robot_name_ + "/dvl";
    std::string pressure_topic = "/" + robot_name_ + "/pressure";

    imu_sub_ = nh_.subscribe(imu_topic, 100, &SensorFrontend::imuCallback, this);
    dvl_sub_ = nh_.subscribe(dvl_topic, 100, &SensorFrontend::dvlCallback, this);
    pressure_sub_ = nh_.subscribe(pressure_topic, 100, &SensorFrontend::pressureCallback, this);

    // Debug publisher for DVL velocity in base_link frame
    dvl_debug_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/" + robot_name_ + "/dvl_body_velocity", 10);

    latest_gyro_.setZero();

    ROS_INFO_STREAM("SensorFrontend initialized for robot: " << robot_name_);
    ROS_INFO_STREAM("Subscribed to IMU: " << imu_topic);
    ROS_INFO_STREAM("Subscribed to DVL: " << dvl_topic);
    ROS_INFO_STREAM("Subscribed to Pressure: " << pressure_topic);
}

void SensorFrontend::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!imu_cb_) return;

    ImuMeasurement m;
    m.timestamp = msg->header.stamp.toSec();
    m.acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    m.gyro = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // IMU orientation (from sensor fusion in Gazebo)
    m.orientation = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                        msg->orientation.y, msg->orientation.z);
    m.has_orientation = true;

    // Cache gyro for DVL compensation
    latest_gyro_ = m.gyro;
    latest_gyro_time_ = m.timestamp;

    imu_cb_(m);
}

void SensorFrontend::dvlCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
    if (!dvl_cb_) return;

    // DVL reports velocity in DVL link frame (dvl_link has 90 deg pitch relative to base_link)
    // We need to transform velocity from dvl_link to base_link

    std::string base_frame = robot_name_ + "/base_link";
    std::string dvl_frame = msg->header.frame_id;

    try {
        // Get transform from dvl_link to base_link using latest available transform (Time(0))
        // This avoids strict timestamp synchronization issues with static transforms in simulation
        if (!tf_buffer_.canTransform(base_frame, dvl_frame, ros::Time(0), ros::Duration(1.0))) {
            ROS_WARN_THROTTLE(5.0, "Waiting for DVL TF transform: %s -> %s",
                              dvl_frame.c_str(), base_frame.c_str());
            return;
        }

        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            base_frame, dvl_frame, ros::Time(0));

        // Extract rotation from transform
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 R(q);

        // DVL velocity in dvl_link frame
        tf2::Vector3 v_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);

        // Rotate to base_link frame
        tf2::Vector3 v_base_meas = R * v_dvl;

        // --- Lever Arm Compensation ---
        // DVL measures velocity at its mounting location, which includes rotation-induced velocity.
        // v_sensor = v_center + omega x r
        // Therefore: v_center = v_sensor - omega x r
        
        // 1. Get lever arm (r) from transform (translation component)
        tf2::Vector3 r_lever(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );

        // 2. Get latest angular velocity (omega)
        // Ensure latest_gyro_ is valid (check timestamp if needed, but high-rate IMU is usually fine)
        tf2::Vector3 omega(latest_gyro_.x(), latest_gyro_.y(), latest_gyro_.z());

        // 3. Calculate tangential velocity: omega x r
        tf2::Vector3 v_tangential = omega.cross(r_lever);

        // 4. Compensate
        tf2::Vector3 v_center = v_base_meas - v_tangential;

        Eigen::Vector3d v_meas(v_center.x(), v_center.y(), v_center.z());

        // Debug output (throttled)
        ROS_DEBUG_THROTTLE(2.0, "DVL Comp: Meas=[%.2f, %.2f, %.2f], Tang=[%.2f, %.2f, %.2f] -> Center=[%.2f, %.2f, %.2f]",
                           v_base_meas.x(), v_base_meas.y(), v_base_meas.z(),
                           v_tangential.x(), v_tangential.y(), v_tangential.z(),
                           v_meas.x(), v_meas.y(), v_meas.z());

        DvlMeasurement m;
        m.timestamp = msg->header.stamp.toSec();
        m.velocity = v_meas;
        dvl_cb_(m);

        // Publish debug message for DVL velocity verification
        geometry_msgs::Vector3Stamped debug_msg;
        debug_msg.header = msg->header;
        debug_msg.header.frame_id = robot_name_ + "/base_link";
        debug_msg.vector.x = v_meas.x();
        debug_msg.vector.y = v_meas.y();
        debug_msg.vector.z = v_meas.z();
        dvl_debug_pub_.publish(debug_msg);

    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(5.0, "DVL TF Error: %s", ex.what());
    }
}

void SensorFrontend::pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
    if (!depth_cb_) return;

    DepthMeasurement m;
    m.timestamp = msg->header.stamp.toSec();
    m.depth = pressureToDepth(msg->fluid_pressure);

    depth_cb_(m);
}

double SensorFrontend::pressureToDepth(double pressure) const {
    if (pressure < ATM_PRESSURE) return 0.0;
    // Calculate sensor depth (ENU: Z-up, underwater is negative Z)
    double sensor_depth = -((pressure - ATM_PRESSURE) / RHO_G);
    // Compensate for sensor mounting offset: sensor is 0.85m above base_link
    // base_link_depth = sensor_depth - offset (deeper = more negative)
    return sensor_depth - PRESSURE_SENSOR_Z_OFFSET;
}

} // namespace rexrov_fgo_nav