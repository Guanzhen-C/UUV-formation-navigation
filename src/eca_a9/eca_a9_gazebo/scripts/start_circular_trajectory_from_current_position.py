#!/usr/bin/env python
# 从当前位置出发的圆形轨迹生成器
# 每个AUV从自己的当前位置找到最近的圆形轨迹点，然后开始跟踪

from __future__ import print_function
import rospy
import sys
import numpy as np
import math
from uuv_control_msgs.srv import InitCircularTrajectory
from numpy import pi
from geometry_msgs.msg import Point
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans


class CircularTrajectoryFromCurrentPosition:
    def __init__(self):
        self.current_position = None
        self.current_orientation = None
        self.current_yaw = None
        self.position_received = False
        
        # 订阅当前位置
        self.odom_sub = rospy.Subscriber('pose_gt', Odometry, self.odom_callback)
        
        # 等待位置信息
        rospy.loginfo('Waiting for current position...')
        while not self.position_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        if self.position_received:
            rospy.loginfo('Current pose received: pos=[{:.2f}, {:.2f}, {:.2f}], yaw={:.1f}°'.format(
                self.current_position[0], self.current_position[1], self.current_position[2],
                self.current_yaw * 180 / pi))
        else:
            rospy.logerr('Failed to get current pose!')
            sys.exit(-1)
    
    def odom_callback(self, msg):
        """获取当前位置和朝向"""
        if not self.position_received:
            self.current_position = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]
            
            # 获取当前朝向
            self.current_orientation = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            
            # 转换为yaw角度（弧度）
            euler = tf_trans.euler_from_quaternion(self.current_orientation)
            self.current_yaw = euler[2]  # yaw是第三个元素
            
            self.position_received = True
    
    def find_closest_angle_on_circle(self, center, radius):
        """找到圆周上离当前位置最近的点对应的角度"""
        # 计算当前位置相对于圆心的向量
        dx = self.current_position[0] - center[0]
        dy = self.current_position[1] - center[1]
        
        # 计算角度（弧度）
        angle = math.atan2(dy, dx)
        
        # 确保角度在[0, 2π]范围内
        if angle < 0:
            angle += 2 * pi
        
        # 计算圆周上最近点的坐标
        closest_x = center[0] + radius * math.cos(angle)
        closest_y = center[1] + radius * math.sin(angle)
        
        rospy.loginfo('Closest point on circle: x={:.2f}, y={:.2f}, angle={:.2f} deg'.format(
            closest_x, closest_y, angle * 180 / pi))
        
        return angle
    
    def find_start_angle_with_tangent_heading(self, center, radius):
        """找到起始角度，确保AUV朝向为该点的切线方向"""
        # 计算当前位置相对于圆心的角度
        dx = self.current_position[0] - center[0]
        dy = self.current_position[1] - center[1]
        position_angle = math.atan2(dy, dx)
        
        # 确保角度在[0, 2π]范围内
        if position_angle < 0:
            position_angle += 2 * pi
        
        # 计算该点的切线方向（圆形轨迹的运动方向）
        # 对于逆时针圆形轨迹，切线方向是该点角度 + π/2
        # 但需要确保方向向量正确
        tangent_heading = position_angle + pi/2
        
        # 标准化角度到[0, 2π]范围
        while tangent_heading > 2 * pi:
            tangent_heading -= 2 * pi
        while tangent_heading < 0:
            tangent_heading += 2 * pi
        
        # 计算圆周上的点坐标
        circle_x = center[0] + radius * math.cos(position_angle)
        circle_y = center[1] + radius * math.sin(position_angle)
        
        rospy.loginfo('Start point on circle: pos=[{:.2f}, {:.2f}], angle={:.1f}°'.format(
            circle_x, circle_y, position_angle * 180 / pi))
        rospy.loginfo('AUV will head in tangent direction: {:.1f}°'.format(
            tangent_heading * 180 / pi))
        
        return position_angle
    
    def generate_trajectory(self, params):
        """生成从最近点开始的圆形轨迹"""
        center = params['center']
        radius = params['radius']
        
        # 找到起始角度，AUV将以切线方向进入轨迹
        start_angle = self.find_start_angle_with_tangent_heading(center, radius)
        
        # 计算切线方向的heading_offset
        # 对于逆时针圆形轨迹，切线方向是起始角度 + π/2
        tangent_heading = start_angle + pi/2
        
        # 标准化角度到[0, 2π]范围
        while tangent_heading > 2 * pi:
            tangent_heading -= 2 * pi
        while tangent_heading < 0:
            tangent_heading += 2 * pi
        
        # 将切线朝向转换为heading_offset（度）
        heading_offset_rad = tangent_heading
        heading_offset_deg = heading_offset_rad * 180 / pi
        
        rospy.loginfo('Calculated heading_offset: {:.1f}° (tangent direction)'.format(heading_offset_deg))
        
        # 使用找到的角度作为起始偏移
        try:
            rospy.wait_for_service('start_circular_trajectory', timeout=20)
        except rospy.ROSException:
            rospy.logerr('Service not available! Closing node...')
            return False

        try:
            traj_gen = rospy.ServiceProxy('start_circular_trajectory', InitCircularTrajectory)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed, error={}'.format(e))
            return False

        # 获取起始时间
        start_time = rospy.Time.now().to_sec()
        start_now = False
        if rospy.has_param('~start_time'):
            start_time = rospy.get_param('~start_time')
            if start_time < 0.0:
                start_time = 0.0
                start_now = True
        else:
            start_now = True

        rospy.loginfo('Generating circular trajectory with start angle={:.1f}° (AUV will face tangent direction)'.format(
            start_angle * 180 / pi))

        # 生成轨迹，使用计算出的起始角度和切线朝向
        success = traj_gen(Time(rospy.Time(start_time)),
                          start_now,
                          params['radius'],
                          Point(params['center'][0], params['center'][1], params['center'][2]),
                          False,
                          start_angle,  # 使用计算出的起始角度
                          params['n_points'],
                          heading_offset_rad,  # 使用计算出的切线朝向
                          params['max_forward_speed'],
                          params['duration'])

        if success:
            rospy.loginfo('Trajectory successfully generated!')
            return True
        else:
            rospy.logerr('Failed to generate trajectory')
            return False


if __name__ == '__main__':
    print('Starting the circular trajectory creator from current position')
    rospy.init_node('start_circular_trajectory_from_current_position')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    # 获取参数
    param_labels = ['radius', 'center', 'n_points', 'heading_offset',
                    'duration', 'max_forward_speed']
    params = dict()

    for label in param_labels:
        if not rospy.has_param('~' + label):
            print('{} must be provided for the trajectory generation!'.format(label))
            sys.exit(-1)
        params[label] = rospy.get_param('~' + label)

    if len(params['center']) != 3:
        print('Center of circle must have 3 components (x, y, z)')
        sys.exit(-1)

    if params['n_points'] <= 2:
        print('Number of points must be at least 2')
        sys.exit(-1)

    if params['max_forward_speed'] <= 0:
        print('Velocity limit must be positive')
        sys.exit(-1)

    # 创建轨迹生成器并生成轨迹
    trajectory_generator = CircularTrajectoryFromCurrentPosition()
    success = trajectory_generator.generate_trajectory(params)
    
    if not success:
        sys.exit(-1)
