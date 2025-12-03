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

    // NDT配准参数 - 优化以提高小位移场景的配准精度
    ndt_.setTransformationEpsilon(0.001);  // 更严格的收敛条件
    ndt_.setStepSize(0.1);   // 更小的步长，避免跳过最优解
    ndt_.setResolution(0.5); // 更细的分辨率，提高小位移精度
    ndt_.setMaximumIterations(50);  // 更多迭代次数确保收敛

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
    
    // [FIX] Hardcoded Static Transform (Optical Frame -> Base Link)
    // Bypassing TF listener due to persistent sync/tree issues in simulation.
    // Transform derived from URDF:
    // Base -> Sonar: xyz(0,0,-0.5) rpy(0, 1.57, 0)
    // Sonar -> Optical: xyz(0,0,0) rpy(-1.57, 0, -1.57)
    // Resulting Base -> Optical Rotation Matrix:
    //  0 -1  0
    // -1  0  0
    //  0  0 -1
    // Translation (in Base frame): (0, 0, -0.5)
    
    Eigen::Affine3d transform_base_optical = Eigen::Affine3d::Identity();
    Eigen::Matrix3d R_base_optical;
    R_base_optical <<  0, -1,  0,
                      -1,  0,  0,
                       0,  0, -1;
    transform_base_optical.linear() = R_base_optical;
    transform_base_optical.translation() << 0.0, 0.0, -0.5;

    // We need Optical -> Base (which is Inverse of Base -> Optical if transforming points?)
    // Actually pcl::transformPointCloud(input, output, transform) applies: output = transform * input
    // So we need Transform_Base_from_Optical (Pose of Optical in Base frame).
    // Wait, standard definition: T_A_B transforms point from B to A.
    // So we need T_Base_Optical. Yes.
    
    Eigen::Affine3d transform_eigen = transform_base_optical;

    pcl::transformPointCloud(*current_cloud_optical, *current_cloud_base, transform_eigen);
    current_cloud_base->header.frame_id = base_frame; // Update frame ID

    /* TF Lookup Commented Out
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
    */

    // Downsample for performance
    PointCloud::Ptr filtered_cloud(new PointCloud());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(current_cloud_base);
    sor.setLeafSize(0.1f, 0.1f, 0.1f); // Revert to 0.1m for better accuracy
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
    double fitness_score = ndt_.getFitnessScore();

    ROS_WARN_THROTTLE(2.0, "NDT Converged: Trans=%.3f m, Rot=%.3f rad, Score=%.3f", trans, rot.angle(), fitness_score);

    // ===== 异常值检测 =====
    // 1. 平移量过大检测 (AUV速度约1m/s，深度相机帧率约10Hz，正常帧间位移应<0.3m)
    const double max_trans_threshold = 0.5;  // 最大允许平移量(米)
    // 2. 旋转量过大检测 (正常旋转应很小)
    const double max_rot_threshold = 0.2;    // 最大允许旋转量(弧度，约11度)
    // 3. Fitness Score过大检测 (值越小配准越好)
    const double max_fitness_threshold = 2.0;  // 最大允许fitness score

    bool is_outlier = false;
    std::string reject_reason;

    if (trans > max_trans_threshold) {
        is_outlier = true;
        reject_reason = "translation too large";
    } else if (rot.angle() > max_rot_threshold) {
        is_outlier = true;
        reject_reason = "rotation too large";
    } else if (fitness_score > max_fitness_threshold) {
        is_outlier = true;
        reject_reason = "fitness score too high";
    }

    if (is_outlier) {
        ROS_WARN("NDT OUTLIER REJECTED: %s (trans=%.3f, rot=%.3f, score=%.3f)",
                 reject_reason.c_str(), trans, rot.angle(), fitness_score);
        return;  // 拒绝这个测量
    }

    // Callback to Factor Graph
    if (odom_callback_) {
        // ===== 动态协方差调整 =====
        // 根据fitness score调整协方差，score越大越不可信
        double base_cov = 0.1;
        double cov_scale = 1.0 + fitness_score;  // fitness越大，协方差越大（越不可信）

        // 如果平移或旋转接近阈值，也增加协方差
        double trans_ratio = trans / max_trans_threshold;
        double rot_ratio = rot.angle() / max_rot_threshold;
        cov_scale *= (1.0 + std::max(trans_ratio, rot_ratio));

        double adjusted_cov = base_cov * cov_scale;

        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
        cov(0, 0) = adjusted_cov;  // roll
        cov(1, 1) = adjusted_cov;  // pitch
        cov(2, 2) = adjusted_cov;  // yaw
        cov(3, 3) = adjusted_cov;  // x
        cov(4, 4) = adjusted_cov;  // y
        cov(5, 5) = adjusted_cov;  // z

        ROS_DEBUG("Sonar cov adjusted: base=%.2f, scale=%.2f, final=%.2f",
                  base_cov, cov_scale, adjusted_cov);

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
