#include "rexrov_fgo_nav/SonarProcessor.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.h>
#include <cmath>

namespace rexrov_fgo_nav {

SonarProcessor::SonarProcessor(ros::NodeHandle& nh, const std::string& robot_name)
    : nh_(nh), robot_name_(robot_name)
{
    // Subscribe only to depth image (intensity topics have timestamp issues)
    std::string depth_topic = "/" + robot_name_ + "/depth/image_raw";

    depth_only_sub_ = nh_.subscribe(depth_topic, 1, &SonarProcessor::depthCallback, this);

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mbes_cloud", 1);
    odom_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mbes_odom", 1);

    // NDT配准参数
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.5);
    ndt_.setResolution(2.0);
    ndt_.setMaximumIterations(30);

    prev_keyframe_cloud_.reset(new PointCloud());
    last_keyframe_pose_ = Eigen::Affine3d::Identity();

    ROS_INFO("SonarProcessor (Depth Camera) initialized. Subscribed to: %s", depth_topic.c_str());
}

void SonarProcessor::depthCallback(const sensor_msgs::Image::ConstPtr& depth_msg) {

    PointCloud::Ptr current_cloud_optical(new PointCloud());
    constructPointCloudFromDepth(depth_msg, current_cloud_optical);

    if (current_cloud_optical->empty()) return;

    // Transform cloud to base_link frame
    PointCloud::Ptr current_cloud_base(new PointCloud());
    std::string base_frame = robot_name_ + "/base_link";
    
    try {
        // Wait for transform using Time(0) to get the latest available static transform
        if (!tf_buffer_.canTransform(base_frame, depth_msg->header.frame_id, ros::Time(0), ros::Duration(0.1))) {
             ROS_WARN_THROTTLE(2.0, "SonarProcessor: Waiting for TF %s -> %s", 
                               depth_msg->header.frame_id.c_str(), base_frame.c_str());
             return;
        }

        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
            base_frame, depth_msg->header.frame_id, ros::Time(0));

        // Use PCL ROS to transform point cloud
        // Note: pcl_ros::transformPointCloud uses tf::Transform, but we have geometry_msgs.
        // Better to use Eigen transform for PCL.
        
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);
        pcl::transformPointCloud(*current_cloud_optical, *current_cloud_base, transform_eigen);
        current_cloud_base->header.frame_id = base_frame; // Update frame ID

    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(2.0, "SonarProcessor TF Error: %s", ex.what());
        return;
    }

    // Downsample for performance
    PointCloud::Ptr filtered_cloud(new PointCloud());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(current_cloud_base);
    sor.setLeafSize(0.1f, 0.1f, 0.1f); // Reverted to 0.1 to keep more points (original value)
    sor.filter(*filtered_cloud);

    // Publish filtered cloud to reduce RViz load

    ROS_WARN_THROTTLE(2.0, "Sonar cloud size: raw=%zu, filtered=%zu", current_cloud_base->size(), filtered_cloud->size());

    if (prev_keyframe_cloud_->empty()) {
        *prev_keyframe_cloud_ = *filtered_cloud;
        ROS_INFO("Initialized first MBES keyframe (Points: %zu) in %s.", filtered_cloud->size(), base_frame.c_str());
        return;
    }

    // if (filtered_cloud->size() < 50) {
    //     ROS_WARN_THROTTLE(5.0, "Sonar cloud too small after filter: %zu", filtered_cloud->size());
    //     return;
    // }

    ndt_.setInputSource(filtered_cloud);
    ndt_.setInputTarget(prev_keyframe_cloud_);

    PointCloud::Ptr output_cloud(new PointCloud());
    ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());

    if (!ndt_.hasConverged()) {
        ROS_WARN_THROTTLE(1.0, "NDT did not converge. Iterations: %d, Score: %f", 
                          ndt_.getFinalNumIteration(), ndt_.getFitnessScore());
        return;
    }

    Eigen::Matrix4f transformation = ndt_.getFinalTransformation();
    Eigen::Affine3d transform_d;
    transform_d.matrix() = transformation.cast<double>();

    double trans = transform_d.translation().norm();
    Eigen::AngleAxisd rot(transform_d.rotation());
    
    ROS_WARN_THROTTLE(2.0, "NDT Converged: Trans=%.3f m, Rot=%.3f rad", trans, rot.angle());

    // Callback to Factor Graph
    if (odom_callback_) {
        // Fixed covariance for sonar odometry
        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
        cov(0, 0) = 0.1;  // roll
        cov(1, 1) = 0.1;  // pitch
        cov(2, 2) = 0.1;  // yaw
        cov(3, 3) = 0.1;  // x
        cov(4, 4) = 0.1;  // y
        cov(5, 5) = 0.1;  // z

        odom_callback_(transform_d, cov, depth_msg->header.stamp.toSec());
    } else {
        ROS_WARN_THROTTLE(5.0, "Odom callback is not set!");
    }

    double trans_norm = transform_d.translation().norm();
    Eigen::AngleAxisd rot_norm(transform_d.rotation());

    geometry_msgs::PoseWithCovarianceStamped odom_msg;
    odom_msg.header = depth_msg->header;
    odom_msg.header.frame_id = "world"; 

    last_keyframe_pose_ = last_keyframe_pose_ * transform_d;

    odom_msg.pose.pose.position.x = last_keyframe_pose_.translation().x();
    odom_msg.pose.pose.position.y = last_keyframe_pose_.translation().y();
    odom_msg.pose.pose.position.z = last_keyframe_pose_.translation().z();
    Eigen::Quaterniond q(last_keyframe_pose_.rotation());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_pub_.publish(odom_msg);

    if (trans > min_translation_update_ || rot.angle() > min_rotation_update_) {
        ROS_INFO("New MBES Keyframe Added. Trans: %.2f, Rot: %.2f", trans, rot.angle());
        PointCloud::Ptr target_cloud(new PointCloud());
        sor.setInputCloud(current_cloud_base);
        sor.filter(*target_cloud);
        *prev_keyframe_cloud_ = *target_cloud;
    }
}

void SonarProcessor::constructPointCloudFromDepth(const sensor_msgs::Image::ConstPtr& depth_msg,
                                                   PointCloud::Ptr& out_cloud) {
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    const cv::Mat& depth_img = depth_ptr->image;

    int rows = depth_img.rows;
    int cols = depth_img.cols;

    double fov = params_.horizontal_fov;
    double focal_length = (double)cols / (2.0 * tan(fov / 2.0));
    double cx = cols / 2.0;
    double cy = rows / 2.0;

    out_cloud->header.frame_id = depth_msg->header.frame_id;
    out_cloud->reserve(rows * cols);

    int valid_points = 0;
    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            float d = depth_img.at<float>(v, u);

            // Skip invalid depth values
            if (std::isnan(d) || d < params_.min_range || d > params_.max_range) continue;

            double x = (u - cx) * d / focal_length;
            double y = (v - cy) * d / focal_length;
            double z = d;

            PointT p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = 100.0f;  // Default intensity

            out_cloud->points.push_back(p);
            valid_points++;
        }
    }
    out_cloud->width = out_cloud->points.size();
    out_cloud->height = 1;
    out_cloud->is_dense = false;

    ROS_DEBUG_THROTTLE(1.0, "SonarProcessor: Depth img size %dx%d, Valid points: %d, Depth sample (center): %.2f", 
                       cols, rows, valid_points, depth_img.at<float>(rows/2, cols/2));
}

} // namespace rexrov_fgo_nav
