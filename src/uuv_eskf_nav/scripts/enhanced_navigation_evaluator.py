#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import tf.transformations as tft

class EnhancedNavigationEvaluator:
    """
    增强版ESKF导航评估器
    功能：
    1. 实时计算位置、速度、姿态误差
    2. 统计误差趋势和分布
    3. 评估导航收敛性和稳定性
    4. 发布详细的性能指标
    """
    def __init__(self):
        # 初始化节点
        rospy.init_node('enhanced_navigation_evaluator', anonymous=True)
        rospy.loginfo("增强版ESKF导航评估器启动")

        # 参数配置
        self.robot_name = rospy.get_param('robot_name', 'eca_a9')  # 默认机器人名称
        if not self.robot_name:
            rospy.logwarn("robot_name参数未设置，使用默认值: 'eca_a9'")
            self.robot_name = 'eca_a9'
        self.filtered_odom_topic = rospy.get_param('~filtered_odom_topic', '/eskf/odometry/filtered')
        self.ground_truth_topic = rospy.get_param('~ground_truth_topic', '/%s/pose_gt' % self.robot_name)
        
        rospy.loginfo("评估器配置:")
        rospy.loginfo("  机器人: %s", self.robot_name)
        rospy.loginfo("  ESKF导航结果话题: %s", self.filtered_odom_topic)
        rospy.loginfo("  地面真值话题: %s", self.ground_truth_topic)

        # 数据存储
        self.gt_pose = None
        self.filtered_pose = None
        self.gt_twist = None
        self.filtered_twist = None
        
        # 误差统计
        self.position_errors = []
        self.velocity_errors = []
        self.attitude_errors = []
        self.timestamps = []
        
        # 性能指标
        self.max_position_error = 0.0
        self.max_velocity_error = 0.0
        self.max_attitude_error = 0.0
        self.avg_position_error = 0.0
        self.avg_velocity_error = 0.0
        self.avg_attitude_error = 0.0
        
        # 收敛性分析
        self.convergence_threshold = 1.0  # 米
        self.is_converged = False
        self.convergence_time = None
        self.start_time = rospy.Time.now()

        # 订阅器
        rospy.Subscriber(self.filtered_odom_topic, Odometry, self.filtered_callback)
        rospy.Subscriber(self.ground_truth_topic, Odometry, self.gt_callback)

        # 发布器 - 发布详细的误差信息
        self.error_pub = rospy.Publisher('/eskf/navigation_errors', Float64MultiArray, queue_size=10)
        
        # 定时器 - 每秒计算和显示误差
        rospy.Timer(rospy.Duration(1.0), self.evaluate_navigation)
        
        # 详细报告定时器 - 每10秒发布详细统计
        rospy.Timer(rospy.Duration(10.0), self.publish_detailed_report)

        rospy.loginfo("ESKF导航评估器初始化完成")
        rospy.loginfo("提示: 可以通过Ctrl+C停止评估器")

    def filtered_callback(self, msg):
        """ESKF滤波结果回调"""
        self.filtered_pose = msg.pose.pose
        self.filtered_twist = msg.twist.twist

    def gt_callback(self, msg):
        """地面真值回调"""
        self.gt_pose = msg.pose.pose
        self.gt_twist = msg.twist.twist

    def quaternion_to_euler(self, q):
        """四元数转欧拉角"""
        return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def calculate_position_error(self):
        """计算位置误差 (欧氏距离)"""
        if not (self.gt_pose and self.filtered_pose):
            return None
            
        gt_pos = np.array([
            self.gt_pose.position.x,
            self.gt_pose.position.y,
            self.gt_pose.position.z
        ])
        
        filt_pos = np.array([
            self.filtered_pose.position.x,
            self.filtered_pose.position.y,
            self.filtered_pose.position.z
        ])
        
        return np.linalg.norm(gt_pos - filt_pos)

    def calculate_velocity_error(self):
        """计算速度误差"""
        if not (self.gt_twist and self.filtered_twist):
            return None
            
        gt_vel = np.array([
            self.gt_twist.linear.x,
            self.gt_twist.linear.y,
            self.gt_twist.linear.z
        ])
        
        filt_vel = np.array([
            self.filtered_twist.linear.x,
            self.filtered_twist.linear.y,
            self.filtered_twist.linear.z
        ])
        
        return np.linalg.norm(gt_vel - filt_vel)

    def calculate_attitude_error(self):
        """计算姿态误差 (角度差异)"""
        if not (self.gt_pose and self.filtered_pose):
            return None
            
        # 转换为欧拉角
        gt_euler = self.quaternion_to_euler(self.gt_pose.orientation)
        filt_euler = self.quaternion_to_euler(self.filtered_pose.orientation)
        
        # 计算角度差异 (弧度)
        angle_diff = np.array(gt_euler) - np.array(filt_euler)
        
        # 处理角度环绕问题
        angle_diff = np.array([self.normalize_angle(diff) for diff in angle_diff])
        
        # 返回角度误差的模长 (转换为度)
        return np.linalg.norm(angle_diff) * 180.0 / np.pi

    def normalize_angle(self, angle):
        """归一化角度到 [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def evaluate_navigation(self, event):
        """主要评估函数"""
        if not (self.gt_pose and self.filtered_pose):
            rospy.logwarn_throttle(6, "等待ESKF导航结果和地面真值数据...")
            return

        # 计算各项误差
        pos_error = self.calculate_position_error()
        vel_error = self.calculate_velocity_error()
        att_error = self.calculate_attitude_error()

        if pos_error is None or vel_error is None or att_error is None:
            return

        # 存储误差数据
        current_time = rospy.Time.now()
        self.position_errors.append(pos_error)
        self.velocity_errors.append(vel_error)
        self.attitude_errors.append(att_error)
        self.timestamps.append(current_time.to_sec())

        # 更新性能指标
        self.max_position_error = max(self.max_position_error, pos_error)
        self.max_velocity_error = max(self.max_velocity_error, vel_error)
        self.max_attitude_error = max(self.max_attitude_error, att_error)

        if len(self.position_errors) > 0:
            self.avg_position_error = np.mean(self.position_errors)
            self.avg_velocity_error = np.mean(self.velocity_errors)
            self.avg_attitude_error = np.mean(self.attitude_errors)

        # 收敛性分析
        if not self.is_converged and pos_error < self.convergence_threshold:
            self.is_converged = True
            self.convergence_time = (current_time - self.start_time).to_sec()
            rospy.loginfo("🎉 导航系统已收敛! 收敛时间: %.1f秒", self.convergence_time)

        # 发布误差数据
        error_msg = Float64MultiArray()
        error_msg.data = [pos_error, vel_error, att_error, 
                         self.avg_position_error, self.avg_velocity_error, self.avg_attitude_error]
        self.error_pub.publish(error_msg)

        # 实时显示
        rospy.loginfo("============================================================")
        rospy.loginfo("ESKF导航系统实时性能评估")
        rospy.loginfo("------------------------------------------------------------")
        rospy.loginfo("位置误差:     %.4f 米", pos_error)
        rospy.loginfo("速度误差:     %.4f 米/秒", vel_error)
        rospy.loginfo("姿态误差:     %.3f 度", att_error)
        rospy.loginfo("------------------------------------------------------------")
        rospy.loginfo("当前ESKF估计状态:")
        rospy.loginfo("  位置: [%.3f, %.3f, %.3f]", 
                     self.filtered_pose.position.x,
                     self.filtered_pose.position.y, 
                     self.filtered_pose.position.z)
        rospy.loginfo("  速度: [%.3f, %.3f, %.3f]", 
                     self.filtered_twist.linear.x,
                     self.filtered_twist.linear.y,
                     self.filtered_twist.linear.z)
        # 估计姿态（欧拉角，度）
        filt_r, filt_p, filt_y = self.quaternion_to_euler(self.filtered_pose.orientation)
        rospy.loginfo("  姿态(欧拉角, 度): [R=%.3f, P=%.3f, Y=%.3f]",
                      math.degrees(filt_r), math.degrees(filt_p), math.degrees(filt_y))

        # 打印AUV地面真值状态
        rospy.loginfo("当前AUV真实状态:")
        rospy.loginfo("  位置: [%.3f, %.3f, %.3f]",
                     self.gt_pose.position.x,
                     self.gt_pose.position.y,
                     self.gt_pose.position.z)
        rospy.loginfo("  速度: [%.3f, %.3f, %.3f]",
                     self.gt_twist.linear.x,
                     self.gt_twist.linear.y,
                     self.gt_twist.linear.z)
        # 真实姿态（欧拉角，度）
        gt_r, gt_p, gt_y = self.quaternion_to_euler(self.gt_pose.orientation)
        rospy.loginfo("  姿态(欧拉角, 度): [R=%.3f, P=%.3f, Y=%.3f]",
                      math.degrees(gt_r), math.degrees(gt_p), math.degrees(gt_y))

    def publish_detailed_report(self, event):
        """发布详细的性能报告"""
        if len(self.position_errors) == 0:
            return

        runtime = (rospy.Time.now() - self.start_time).to_sec()
        
        # 计算统计指标
        pos_std = np.std(self.position_errors) if len(self.position_errors) > 1 else 0.0
        vel_std = np.std(self.velocity_errors) if len(self.velocity_errors) > 1 else 0.0
        att_std = np.std(self.attitude_errors) if len(self.attitude_errors) > 1 else 0.0
        
        rospy.loginfo("================================================================")
        rospy.loginfo("📊 ESKF导航系统性能统计报告 (运行时间: %.1f秒)", runtime)
        rospy.loginfo("================================================================")
        rospy.loginfo("位置误差统计:")
        rospy.loginfo("  平均值: %.4f 米    最大值: %.4f 米    标准差: %.4f 米", 
                     self.avg_position_error, self.max_position_error, pos_std)
        rospy.loginfo("速度误差统计:")
        rospy.loginfo("  平均值: %.4f 米/秒  最大值: %.4f 米/秒  标准差: %.4f 米/秒", 
                     self.avg_velocity_error, self.max_velocity_error, vel_std)
        rospy.loginfo("姿态误差统计:")
        rospy.loginfo("  平均值: %.3f 度     最大值: %.3f 度     标准差: %.3f 度", 
                     self.avg_attitude_error, self.max_attitude_error, att_std)
        
        if self.is_converged:
            rospy.loginfo("收敛状态: ✅ 已收敛 (%.1f秒)", self.convergence_time)
        else:
            rospy.loginfo("收敛状态: ⏳ 未收敛 (阈值: %.1f米)", self.convergence_threshold)
        
        rospy.loginfo("================================================================")

if __name__ == '__main__':
    try:
        EnhancedNavigationEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass