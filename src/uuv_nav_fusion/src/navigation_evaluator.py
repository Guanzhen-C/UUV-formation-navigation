#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Odometry

class NavigationEvaluator:
    """
    该节点用于比较robot_localization的输出和地面真值，
    并计算它们之间的位置误差。
    """
    def __init__(self):
        # 初始化节点
        rospy.init_node('navigation_evaluator', anonymous=True)
        rospy.loginfo("导航评估节点已启动。")

        # 用于存储收到的最新位姿数据
        self.gt_pose = None
        self.fused_pose = None
        # 用于存储第一次计算时的初始偏移量
        self.initial_offset = None

        # 从参数服务器获取机器人名称
        # 默认使用eca_a9，可通过launch文件或命令行参数覆盖
        self.robot_name = rospy.get_param('~robot_name', 'eca_a9')
        
        # 验证参数值的有效性
        if not self.robot_name or self.robot_name.strip() == '':
            rospy.logfatal("错误：robot_name参数值为空！")
            rospy.signal_shutdown("robot_name参数值无效")
            return

        # 显示配置信息
        rospy.loginfo("导航评估器配置 - 机器人: %s", self.robot_name)

        # 订阅组合导航结果 (/odometry/filtered)
        rospy.Subscriber('/odometry/filtered', Odometry, self.fused_callback)
        # 订阅地面真值 (/{robot_name}/pose_gt)
        rospy.Subscriber('/%s/pose_gt' % self.robot_name, Odometry, self.gt_callback)

        # 创建一个1Hz的定时器，每秒执行一次误差计算
        rospy.Timer(rospy.Duration(1.0), self.calculate_error)

    def fused_callback(self, msg):
        """回调函数：存储最新的组合导航位姿"""
        self.fused_pose = msg.pose.pose

    def gt_callback(self, msg):
        """回调函数：存储最新的地面真值位姿"""
        self.gt_pose = msg.pose.pose

    def calculate_error(self, event):
        """定时器回调函数：计算并打印误差"""
        if self.fused_pose and self.gt_pose:
            # 将地面真值的位置转换为numpy数组
            gt_pos = np.array([
                self.gt_pose.position.x,
                self.gt_pose.position.y,
                self.gt_pose.position.z
            ])

            # 将组合导航结果的位置转换为numpy数组
            fused_pos = np.array([
                self.fused_pose.position.x,
                self.fused_pose.position.y,
                self.fused_pose.position.z
            ])

            # 在第一次收到数据时，计算并存储初始偏移量
            if self.initial_offset is None:
                self.initial_offset = gt_pos - fused_pos
                rospy.loginfo("计算初始偏移量: (%.2f, %.2f, %.2f)", 
                              self.initial_offset[0], self.initial_offset[1], self.initial_offset[2])

            # 从地面真值中减去初始偏移量，以对齐两个坐标系
            aligned_gt_pos = gt_pos - self.initial_offset

            # 计算对齐后的两个位置之间的欧氏距离
            error = np.linalg.norm(aligned_gt_pos - fused_pos)

            # 打印格式化的误差信息
            rospy.loginfo("-----------------------------------------")
            rospy.loginfo("对齐后真值 (odom): (%.2f, %.2f, %.2f)", 
                          aligned_gt_pos[0], aligned_gt_pos[1], aligned_gt_pos[2])
            rospy.loginfo("融合位置 (odom):  (%.2f, %.2f, %.2f)", 
                          fused_pos[0], fused_pos[1], fused_pos[2])
            rospy.loginfo(">>>> 真实漂移误差: %.3f 米", error)
        else:
            # 如果还没有收到两个话题的数据，就发出警告
            rospy.logwarn_throttle(5, "正在等待地面真值和组合导航结果的话题...")

if __name__ == '__main__':
    try:
        NavigationEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 