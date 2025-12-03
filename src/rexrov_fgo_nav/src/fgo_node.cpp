#include <ros/ros.h>
#include "rexrov_fgo_nav/SensorFrontend.h"
#include "rexrov_fgo_nav/SonarProcessor.h"
#include "rexrov_fgo_nav/SlidingWindowGraph.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>

using namespace rexrov_fgo_nav;

int main(int argc, char** argv) {
    ros::init(argc, argv, "rexrov_fgo_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string robot_name;
    pnh.param<std::string>("robot_name", robot_name, "rexrov");

    ROS_INFO("Starting FGO Navigation Node for %s...", robot_name.c_str());

    // 1. Instantiate Components
    SensorFrontend frontend(nh, robot_name);
    SonarProcessor sonar_processor(nh, robot_name);
    SlidingWindowGraph fgo_graph(nh);

    // Publishers for results
    ros::Publisher path_pub = nh.advertise<nav_msgs::Odometry>("fgo_odom", 10);
    ros::Publisher dvl_debug_pub = nh.advertise<geometry_msgs::Vector3Stamped>("dvl_body_velocity", 10);

    // TF Broadcaster for RViz visualization
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Subscribe to ground truth for initialization (pose AND velocity)
    ros::Subscriber gt_sub = nh.subscribe<nav_msgs::Odometry>(
        "/" + robot_name + "/pose_gt", 1,
        [&](const nav_msgs::Odometry::ConstPtr& msg) {
            // Convert to GTSAM Pose3
            gtsam::Point3 pos(msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
            gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
            gtsam::Pose3 gt_pose(rot, pos);
            fgo_graph.setInitialPose(gt_pose);

            // Also set initial velocity from ground truth (world frame)
            gtsam::Vector3 gt_vel(msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y,
                                  msg->twist.twist.linear.z);
            fgo_graph.setInitialVelocity(gt_vel);
        });

    // 2. Connect Callbacks
    
    // IMU -> Graph (High Rate)
    frontend.setImuCallback([&](const ImuMeasurement& m) {
        static double last_imu_time = 0;

        // Pass orientation for initialization
        if (m.has_orientation) {
            gtsam::Rot3 rot = gtsam::Rot3::Quaternion(m.orientation.w(), m.orientation.x(),
                                                       m.orientation.y(), m.orientation.z());
            fgo_graph.addImuOrientation(rot);
        }

        if (last_imu_time > 0) {
            double dt = m.timestamp - last_imu_time;
            if (dt > 0) {
                // Convert Eigen to GTSAM Vector3
                gtsam::Vector3 acc(m.acc.x(), m.acc.y(), m.acc.z());
                gtsam::Vector3 gyro(m.gyro.x(), m.gyro.y(), m.gyro.z());
                fgo_graph.addImuMeasurement(acc, gyro, dt);
            }
        }
        last_imu_time = m.timestamp;
    });

    // DVL -> Graph
    frontend.setDvlCallback([&](const DvlMeasurement& m) {
        gtsam::Vector3 vel(m.velocity.x(), m.velocity.y(), m.velocity.z());
        fgo_graph.addDvlMeasurement(vel, m.timestamp);
    });

    // Depth -> Graph
    frontend.setDepthCallback([&](const DepthMeasurement& m) {
        fgo_graph.addDepthMeasurement(m.depth, m.timestamp);
    });

    // Sonar -> Graph (Scan Matching Result)
    sonar_processor.setOdomCallback([&](const Eigen::Affine3d& rel_pose, const Eigen::Matrix<double, 6, 6>& cov, double ts) {
        gtsam::Pose3 pose(gtsam::Rot3(rel_pose.rotation()), gtsam::Point3(rel_pose.translation()));
        fgo_graph.addSonarRelativePose(pose, cov, ts);
    });

    // 3. Main Loop
    // Use a Timer for the Graph Optimization Loop (increased to ~30Hz for smoother visualization)
    ros::Timer timer = nh.createTimer(ros::Duration(0.033), [&](const ros::TimerEvent&) {
        fgo_graph.update();
        
        // Publish Result
        gtsam::Pose3 current_pose = fgo_graph.getCurrentPose();
        gtsam::Vector3 current_vel = fgo_graph.getCurrentVelocity();
        
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = robot_name + "/base_link";
        
        odom_msg.pose.pose.position.x = current_pose.x();
        odom_msg.pose.pose.position.y = current_pose.y();
        odom_msg.pose.pose.position.z = current_pose.z();
        odom_msg.pose.pose.orientation.w = current_pose.rotation().toQuaternion().w();
        odom_msg.pose.pose.orientation.x = current_pose.rotation().toQuaternion().x();
        odom_msg.pose.pose.orientation.y = current_pose.rotation().toQuaternion().y();
        odom_msg.pose.pose.orientation.z = current_pose.rotation().toQuaternion().z();
        
        odom_msg.twist.twist.linear.x = current_vel.x();
        odom_msg.twist.twist.linear.y = current_vel.y();
        odom_msg.twist.twist.linear.z = current_vel.z();

        path_pub.publish(odom_msg);

        /* 
        // TF Broadcast disabled to prevent conflict (jitter) with Gazebo Ground Truth TF.
        // Users should visualize the estimate using the /fgo_odom topic (Odometry Display).
        
        // Broadcast TF: world -> robot_name/base_link
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "world";
        tf_msg.child_frame_id = robot_name + "/base_link";
        tf_msg.transform.translation.x = current_pose.x();
        tf_msg.transform.translation.y = current_pose.y();
        tf_msg.transform.translation.z = current_pose.z();
        tf_msg.transform.rotation.w = current_pose.rotation().toQuaternion().w();
        tf_msg.transform.rotation.x = current_pose.rotation().toQuaternion().x();
        tf_msg.transform.rotation.y = current_pose.rotation().toQuaternion().y();
        tf_msg.transform.rotation.z = current_pose.rotation().toQuaternion().z();
        tf_broadcaster.sendTransform(tf_msg);
        */
    });

    ros::spin();

    return 0;
}
