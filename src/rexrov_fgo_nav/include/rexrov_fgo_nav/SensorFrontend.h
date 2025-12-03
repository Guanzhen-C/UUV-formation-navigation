#ifndef REXROV_FGO_NAV_SENSOR_FRONTEND_H
#define REXROV_FGO_NAV_SENSOR_FRONTEND_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <sensor_msgs/PointCloud2.h> 

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <functional>
#include <string>
#include <Eigen/Dense>

namespace rexrov_fgo_nav {

struct ImuMeasurement {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Quaterniond orientation;
    bool has_orientation = false;
};

struct DvlMeasurement {
    double timestamp;
    Eigen::Vector3d velocity; // In base_link frame (corrected)
};

struct DepthMeasurement {
    double timestamp;
    double depth;
};

class SensorFrontend {
public:
    using ImuCallback = std::function<void(const ImuMeasurement&)>;
    using DvlCallback = std::function<void(const DvlMeasurement&)>;
    using DepthCallback = std::function<void(const DepthMeasurement&)>;

    SensorFrontend(ros::NodeHandle& nh, const std::string& robot_name);
    ~SensorFrontend() = default;

    void setImuCallback(ImuCallback cb) { imu_cb_ = cb; }
    void setDvlCallback(DvlCallback cb) { dvl_cb_ = cb; }
    void setDepthCallback(DepthCallback cb) { depth_cb_ = cb; }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void dvlCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg);
    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);

    double pressureToDepth(double pressure) const;

    ros::NodeHandle nh_;
    std::string robot_name_;

    ros::Subscriber imu_sub_;
    ros::Subscriber dvl_sub_;
    ros::Subscriber pressure_sub_;

    // Debug publisher for DVL velocity verification
    ros::Publisher dvl_debug_pub_;

    ImuCallback imu_cb_;
    DvlCallback dvl_cb_;
    DepthCallback depth_cb_;
    
    // TF for Lever Arm - Order matters! Buffer must be initialized before Listener.
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Cache for Lever Arm Compensation
    Eigen::Vector3d latest_gyro_;
    double latest_gyro_time_ = 0.0;

    // Constants
    static constexpr double ATM_PRESSURE = 101.325; // kPa
    static constexpr double RHO_G = 9.80638;        // kPa/m approx
    // Pressure sensor is mounted at z=+0.85m above base_link
    // Need to compensate: base_link_depth = sensor_depth - 0.85m
    static constexpr double PRESSURE_SENSOR_Z_OFFSET = 0.85; // m (sensor above base_link)
};

} // namespace rexrov_fgo_nav

#endif // REXROV_FGO_NAV_SENSOR_FRONTEND_H