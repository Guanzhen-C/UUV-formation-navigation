#ifndef REXROV_FGO_NAV_SONAR_PROCESSOR_H
#define REXROV_FGO_NAV_SONAR_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <mutex>
#include <Eigen/Dense>
#include <functional>

namespace rexrov_fgo_nav {

class SonarProcessor {
public:
    using PointT = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<PointT>;
    
    // Callback signature for Odom result
    using OdomCallback = std::function<void(const Eigen::Affine3d&, const Eigen::Matrix<double, 6, 6>&, double)>;

    struct SonarParams {
        double min_range = 0.5;
        double max_range = 150.0; 
        double horizontal_fov = 2.09; 
        double vertical_fov = 2.09;    
        int width = 512;
        int height = 512;
        double intensity_threshold = 10.0;
    };

    SonarProcessor(ros::NodeHandle& nh, const std::string& robot_name);
    ~SonarProcessor() = default;

    void setOdomCallback(OdomCallback cb) { odom_callback_ = cb; }

    PointCloud::Ptr getLastKeyframe() const { return prev_keyframe_cloud_; }

private:
    // Depth-only callback (intensity topics have timestamp issues)
    void depthCallback(const sensor_msgs::Image::ConstPtr& depth_msg);

    // Construct point cloud from depth image only
    void constructPointCloudFromDepth(const sensor_msgs::Image::ConstPtr& depth_msg,
                                       PointCloud::Ptr& out_cloud);
    
    ros::NodeHandle nh_;
    std::string robot_name_;

    // Single depth subscriber (intensity topics have timestamp=0 issue)
    ros::Subscriber depth_only_sub_;

    ros::Publisher cloud_pub_; 
    ros::Publisher odom_pub_; 
    
    OdomCallback odom_callback_;

    SonarParams params_;
    
    // NDT Registration
    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    PointCloud::Ptr prev_keyframe_cloud_;
    
    Eigen::Affine3d last_keyframe_pose_;
    double min_translation_update_ = 1.0;  // meters
    double min_rotation_update_ = 0.1;     // radians (~6°)

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
};

} // namespace rexrov_fgo_nav

#endif // REXROV_FGO_NAV_SONAR_PROCESSOR_H