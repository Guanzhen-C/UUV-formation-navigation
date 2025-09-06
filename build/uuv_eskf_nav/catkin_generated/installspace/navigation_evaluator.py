#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESKF导航评估器

功能:
1. 比较ESKF导航结果与地面真值
2. 计算位置、速度、姿态误差
3. 统计误差分布和收敛性能
4. 提供实时性能监控

作者: ESKF Navigation Team
日期: 2024
"""

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

class ESKFNavigationEvaluator:
    """ESKF导航系统评估器"""
    
    def __init__(self):
        rospy.init_node('eskf_navigation_evaluator', anonymous=True)
        rospy.loginfo("ESKF导航评估器启动")
        
        # 参数读取
        self.robot_name = rospy.get_param('~robot_name', 'eca_a9')
        self.filtered_odom_topic = rospy.get_param('~filtered_odom_topic', '/eskf/odometry/filtered')
        self.ground_truth_topic = rospy.get_param('~ground_truth_topic', f'/{self.robot_name}/pose_gt')
        
        # 验证参数
        if not self.robot_name or self.robot_name.strip() == '':
            rospy.logfatal("错误: robot_name参数值为空!")
            rospy.signal_shutdown("robot_name参数值无效")
            return
            
        rospy.loginfo("评估器配置:")
        rospy.loginfo(f"  机器人: {self.robot_name}")
        rospy.loginfo(f"  ESKF导航结果话题: {self.filtered_odom_topic}")
        rospy.loginfo(f"  地面真值话题: {self.ground_truth_topic}")
        
        # 数据存储
        self.eskf_pose = None
        self.gt_pose = None
        self.eskf_twist = None
        self.gt_twist = None
        
        # 初始偏移量 (用于坐标系对齐)
        self.initial_position_offset = None
        self.initial_orientation_offset = None
        
        # 误差统计
        self.error_history = {
            'position_errors': [],
            'velocity_errors': [], 
            'orientation_errors': [],
            'timestamps': []
        }
        
        # 性能统计
        self.stats = {
            'total_samples': 0,
            'mean_position_error': 0.0,
            'max_position_error': 0.0,
            'mean_velocity_error': 0.0,
            'max_velocity_error': 0.0,
            'mean_orientation_error': 0.0,  # 单位: 度
            'max_orientation_error': 0.0    # 单位: 度
        }
        
        # ROS订阅器
        self.eskf_sub = rospy.Subscriber(self.filtered_odom_topic, Odometry, self.eskf_callback)
        self.gt_sub = rospy.Subscriber(self.ground_truth_topic, Odometry, self.gt_callback)
        
        # 定时器 - 每1秒计算并显示误差
        self.evaluation_timer = rospy.Timer(rospy.Duration(1.0), self.evaluation_callback)
        
        # 统计定时器 - 每30秒显示统计信息
        self.stats_timer = rospy.Timer(rospy.Duration(30.0), self.stats_callback)
        
        rospy.loginfo("ESKF导航评估器初始化完成")
    
    def eskf_callback(self, msg):
        """ESKF导航结果回调"""
        self.eskf_pose = msg.pose.pose
        self.eskf_twist = msg.twist.twist
    
    def gt_callback(self, msg):
        """地面真值回调"""
        self.gt_pose = msg.pose.pose  
        self.gt_twist = msg.twist.twist
    
    def evaluation_callback(self, event):
        """评估定时器回调 - 计算和显示误差"""
        if not self.eskf_pose or not self.gt_pose:
            rospy.logwarn_throttle(5, "等待ESKF导航结果和地面真值数据...")
            return
            
        # 计算误差
        try:
            position_error, velocity_error, orientation_error = self.calculate_errors()
            
            # 更新统计
            self.update_statistics(position_error, velocity_error, orientation_error)
            
            # 显示实时误差
            self.display_realtime_errors(position_error, velocity_error, orientation_error)
            
            # 保存误差历史 (用于后续分析)
            self.save_error_history(position_error, velocity_error, orientation_error)
            
        except Exception as e:
            rospy.logwarn(f"误差计算失败: {e}")
    
    def calculate_errors(self):
        """计算位置、速度、姿态误差"""
        
        # 1. 位置误差计算
        gt_pos = np.array([
            self.gt_pose.position.x,
            self.gt_pose.position.y, 
            self.gt_pose.position.z
        ])
        
        eskf_pos = np.array([
            self.eskf_pose.position.x,
            self.eskf_pose.position.y,
            self.eskf_pose.position.z  
        ])
        
        # 初次运行时计算初始偏移量 (坐标系对齐)
        if self.initial_position_offset is None:
            self.initial_position_offset = gt_pos - eskf_pos
            rospy.loginfo(f"初始位置偏移: [{self.initial_position_offset[0]:.3f}, "
                         f"{self.initial_position_offset[1]:.3f}, "
                         f"{self.initial_position_offset[2]:.3f}]")
        
        # 对齐坐标系后计算位置误差
        aligned_gt_pos = gt_pos - self.initial_position_offset
        position_error = np.linalg.norm(aligned_gt_pos - eskf_pos)
        
        # 2. 速度误差计算  
        if self.gt_twist and self.eskf_twist:
            gt_vel = np.array([
                self.gt_twist.linear.x,
                self.gt_twist.linear.y,
                self.gt_twist.linear.z
            ])
            
            eskf_vel = np.array([
                self.eskf_twist.linear.x,
                self.eskf_twist.linear.y, 
                self.eskf_twist.linear.z
            ])
            
            velocity_error = np.linalg.norm(gt_vel - eskf_vel)
        else:
            velocity_error = 0.0
        
        # 3. 姿态误差计算 (四元数误差)
        gt_quat = [
            self.gt_pose.orientation.x,
            self.gt_pose.orientation.y,
            self.gt_pose.orientation.z, 
            self.gt_pose.orientation.w
        ]
        
        eskf_quat = [
            self.eskf_pose.orientation.x,
            self.eskf_pose.orientation.y,
            self.eskf_pose.orientation.z,
            self.eskf_pose.orientation.w
        ]
        
        # 初次运行时计算初始姿态偏移量
        if self.initial_orientation_offset is None:
            gt_euler = euler_from_quaternion(gt_quat)
            eskf_euler = euler_from_quaternion(eskf_quat) 
            self.initial_orientation_offset = np.array(gt_euler) - np.array(eskf_euler)
            rospy.loginfo(f"初始姿态偏移 (度): [{math.degrees(self.initial_orientation_offset[0]):.2f}, "
                         f"{math.degrees(self.initial_orientation_offset[1]):.2f}, "
                         f"{math.degrees(self.initial_orientation_offset[2]):.2f}]")
        
        # 计算姿态误差 (欧拉角差值的模长)
        gt_euler = np.array(euler_from_quaternion(gt_quat))
        eskf_euler = np.array(euler_from_quaternion(eskf_quat))
        aligned_gt_euler = gt_euler - self.initial_orientation_offset
        
        # 处理角度的周期性 (-pi到pi)
        euler_diff = aligned_gt_euler - eskf_euler
        euler_diff = np.array([self.normalize_angle(angle) for angle in euler_diff])
        orientation_error = np.linalg.norm(euler_diff)
        
        return position_error, velocity_error, orientation_error
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def update_statistics(self, pos_err, vel_err, ori_err):
        """更新误差统计信息"""
        self.stats['total_samples'] += 1
        n = self.stats['total_samples']
        
        # 增量式均值更新 (避免数值精度问题)
        self.stats['mean_position_error'] += (pos_err - self.stats['mean_position_error']) / n
        self.stats['mean_velocity_error'] += (vel_err - self.stats['mean_velocity_error']) / n  
        self.stats['mean_orientation_error'] += (math.degrees(ori_err) - self.stats['mean_orientation_error']) / n
        
        # 最大误差更新
        self.stats['max_position_error'] = max(self.stats['max_position_error'], pos_err)
        self.stats['max_velocity_error'] = max(self.stats['max_velocity_error'], vel_err)
        self.stats['max_orientation_error'] = max(self.stats['max_orientation_error'], math.degrees(ori_err))
    
    def save_error_history(self, pos_err, vel_err, ori_err):
        """保存误差历史数据"""
        current_time = rospy.get_time()
        
        # 限制历史数据长度 (避免内存占用过大)
        max_history_length = 1000
        if len(self.error_history['timestamps']) >= max_history_length:
            self.error_history['position_errors'].pop(0)
            self.error_history['velocity_errors'].pop(0)
            self.error_history['orientation_errors'].pop(0)
            self.error_history['timestamps'].pop(0)
        
        self.error_history['position_errors'].append(pos_err)
        self.error_history['velocity_errors'].append(vel_err)
        self.error_history['orientation_errors'].append(math.degrees(ori_err))
        self.error_history['timestamps'].append(current_time)
    
    def display_realtime_errors(self, pos_err, vel_err, ori_err):
        """显示实时误差信息"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("ESKF导航系统实时性能评估")
        rospy.loginfo("-" * 60)
        rospy.loginfo(f"位置误差:     {pos_err:.4f} 米")
        rospy.loginfo(f"速度误差:     {vel_err:.4f} 米/秒") 
        rospy.loginfo(f"姿态误差:     {math.degrees(ori_err):.3f} 度")
        
        # 显示当前状态
        if self.eskf_pose:
            rospy.loginfo("-" * 60)
            rospy.loginfo("当前ESKF估计状态:")
            rospy.loginfo(f"  位置: [{self.eskf_pose.position.x:.3f}, "
                         f"{self.eskf_pose.position.y:.3f}, "
                         f"{self.eskf_pose.position.z:.3f}]")
            
            if self.eskf_twist:
                rospy.loginfo(f"  速度: [{self.eskf_twist.linear.x:.3f}, "
                             f"{self.eskf_twist.linear.y:.3f}, "
                             f"{self.eskf_twist.linear.z:.3f}]")
    
    def stats_callback(self, event):
        """统计定时器回调 - 显示长期统计信息"""
        if self.stats['total_samples'] == 0:
            return
            
        rospy.loginfo("=" * 80)
        rospy.loginfo("ESKF导航系统长期性能统计")
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"评估样本数:           {self.stats['total_samples']}")
        rospy.loginfo(f"平均位置误差:         {self.stats['mean_position_error']:.4f} 米")
        rospy.loginfo(f"最大位置误差:         {self.stats['max_position_error']:.4f} 米")
        rospy.loginfo(f"平均速度误差:         {self.stats['mean_velocity_error']:.4f} 米/秒")
        rospy.loginfo(f"最大速度误差:         {self.stats['max_velocity_error']:.4f} 米/秒")
        rospy.loginfo(f"平均姿态误差:         {self.stats['mean_orientation_error']:.3f} 度")
        rospy.loginfo(f"最大姿态误差:         {self.stats['max_orientation_error']:.3f} 度")
        
        # 简单的性能评价
        if self.stats['mean_position_error'] < 0.1:
            rospy.loginfo(">>> 位置估计精度: 优秀 (< 0.1m)")
        elif self.stats['mean_position_error'] < 0.5:
            rospy.loginfo(">>> 位置估计精度: 良好 (< 0.5m)")
        else:
            rospy.logwarn(">>> 位置估计精度: 需要改进 (> 0.5m)")
        
        rospy.loginfo("=" * 80)
    
    def get_error_statistics(self):
        """获取误差统计信息 (供外部调用)"""
        return {
            'stats': self.stats.copy(),
            'history': self.error_history.copy()
        }


def main():
    try:
        evaluator = ESKFNavigationEvaluator()
        
        rospy.loginfo("ESKF导航评估器已启动，开始监控导航性能...")
        rospy.loginfo("提示: 可以通过Ctrl+C停止评估器")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ESKF导航评估器已停止")
    except Exception as e:
        rospy.logerr(f"ESKF导航评估器异常: {e}")


if __name__ == '__main__':
    main()
