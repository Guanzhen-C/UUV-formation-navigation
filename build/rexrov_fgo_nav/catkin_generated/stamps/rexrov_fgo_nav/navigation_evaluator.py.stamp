#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
FGO Navigation Evaluator

Compare FGO navigation output with ground truth and calculate position errors.
Based on the pattern from uuv_nav_fusion/navigation_evaluator.py
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry


class FGONavigationEvaluator:
    """
    Compare FGO navigation output with ground truth,
    calculate and display position errors.
    """
    def __init__(self):
        rospy.init_node('fgo_navigation_evaluator', anonymous=True)
        rospy.loginfo("FGO Navigation Evaluator started.")

        # Store latest poses
        self.gt_pose = None
        self.fgo_pose = None

        # Error statistics
        self.error_history = []
        self.max_history = 100  # Keep last 100 samples for statistics

        # Get parameters
        self.robot_name = rospy.get_param('~robot_name', 'rexrov')
        self.fgo_odom_topic = rospy.get_param('~fgo_odom_topic', '/{}/fgo_odom'.format(self.robot_name))
        self.gt_odom_topic = rospy.get_param('~gt_odom_topic', '/{}/pose_gt'.format(self.robot_name))
        self.rate = rospy.get_param('~rate', 1.0)

        if not self.robot_name or self.robot_name.strip() == '':
            rospy.logfatal("Error: robot_name parameter is empty!")
            rospy.signal_shutdown("Invalid robot_name parameter")
            return

        rospy.loginfo("Configuration - Robot: %s", self.robot_name)
        rospy.loginfo("FGO Odom Topic: %s", self.fgo_odom_topic)
        rospy.loginfo("Ground Truth Topic: %s", self.gt_odom_topic)

        # Subscribe to FGO navigation output
        rospy.Subscriber(self.fgo_odom_topic, Odometry, self.fgo_callback)
        # Subscribe to ground truth
        rospy.Subscriber(self.gt_odom_topic, Odometry, self.gt_callback)

        # Timer for error calculation
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.calculate_error)

    def fgo_callback(self, msg):
        """Store latest FGO pose"""
        self.fgo_pose = msg.pose.pose

    def gt_callback(self, msg):
        """Store latest ground truth pose"""
        self.gt_pose = msg.pose.pose

    def calculate_error(self, event):
        """Calculate and print error statistics"""
        if self.fgo_pose and self.gt_pose:
            # Ground truth position
            gt_pos = np.array([
                self.gt_pose.position.x,
                self.gt_pose.position.y,
                self.gt_pose.position.z
            ])

            # FGO estimated position
            fgo_pos = np.array([
                self.fgo_pose.position.x,
                self.fgo_pose.position.y,
                self.fgo_pose.position.z
            ])

            # Calculate position error (Euclidean distance)
            error_vec = fgo_pos - gt_pos
            error_3d = np.linalg.norm(error_vec)
            error_2d = np.linalg.norm(error_vec[:2])  # XY only

            # Update error history
            self.error_history.append(error_3d)
            if len(self.error_history) > self.max_history:
                self.error_history.pop(0)

            # Calculate statistics
            mean_error = np.mean(self.error_history)
            max_error = np.max(self.error_history)
            std_error = np.std(self.error_history)

            # Print formatted error information
            rospy.loginfo("-" * 50)
            rospy.loginfo("Ground Truth: (%.2f, %.2f, %.2f)",
                          gt_pos[0], gt_pos[1], gt_pos[2])
            rospy.loginfo("FGO Estimate: (%.2f, %.2f, %.2f)",
                          fgo_pos[0], fgo_pos[1], fgo_pos[2])
            rospy.loginfo("Error (X, Y, Z): (%.3f, %.3f, %.3f) m",
                          error_vec[0], error_vec[1], error_vec[2])
            rospy.loginfo(">>>> 3D Error: %.3f m | 2D Error: %.3f m", error_3d, error_2d)
            rospy.loginfo("Statistics - Mean: %.3f m | Max: %.3f m | Std: %.3f m",
                          mean_error, max_error, std_error)
        else:
            # Waiting for data
            missing = []
            if not self.gt_pose:
                missing.append("ground truth")
            if not self.fgo_pose:
                missing.append("FGO output")
            rospy.logwarn_throttle(5, "Waiting for %s...", " and ".join(missing))


if __name__ == '__main__':
    try:
        FGONavigationEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
