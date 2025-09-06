#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped

# 根据物理公式，定义压强转深度的常量 (单位: kPa)
# 仿真器插件输出的压强单位是 kPa
# P_kPa = P_atm_kPa + (rho * g * h)/1000
G = 9.81  # 重力加速度 m/s^2
WATER_DENSITY = 1025.0  # 海水密度 kg/m^3
ATM_PRESSURE_KPA = 101.325 # 标准大气压, 单位: kPa
# 关键修复：直接使用UUV仿真器配置文件中定义的、硬编码的转换系数
KPA_PER_METER = 9.80638

class SensorProcessor:
    """
    一个简洁的传感器数据处理器，用于将UUV的原始传感器数据
    转换为robot_localization包所期望的格式。
    """
    def __init__(self):
        rospy.loginfo("正在初始化传感器处理器节点...")

        # 从参数服务器获取机器人名称和坐标系名称
        # 默认使用eca_a9，可通过launch文件或命令行参数覆盖
        self.robot_name = rospy.get_param('~robot_name', 'eca_a9')
        
        # 验证参数值的有效性
        if not self.robot_name or self.robot_name.strip() == '':
            rospy.logfatal("错误：robot_name参数值为空！")
            rospy.signal_shutdown("robot_name参数值无效")
            return
            
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link_frame', '{}/base_link'.format(self.robot_name))
        # 定义DVL传感器的坐标系名称
        self.dvl_link_frame = '{}/dvl_link'.format(self.robot_name)
        
        # 显示配置信息
        rospy.loginfo("传感器处理器配置 - 机器人: %s", self.robot_name)

        # 初始化发布器
        # IMU数据，robot_localization可以直接使用
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        # DVL数据，转换为TwistWithCovarianceStamped
        self.dvl_pub = rospy.Publisher('dvl/twist', TwistWithCovarianceStamped, queue_size=10)
        # 深度数据，转换为PoseWithCovarianceStamped
        self.depth_pub = rospy.Publisher('depth/pose', PoseWithCovarianceStamped, queue_size=10)

        # 初始化订阅器
        rospy.Subscriber('/{}/imu'.format(self.robot_name), Imu, self.imu_callback)
        rospy.Subscriber('/{}/dvl'.format(self.robot_name), DVL, self.dvl_callback)
        rospy.Subscriber('/{}/pressure'.format(self.robot_name), FluidPressure, self.depth_callback)

        rospy.loginfo("传感器处理器节点初始化完成。")

    def imu_callback(self, msg):
        """
        处理IMU消息。
        ROS消息中的数组是元组(tuple)，不可直接修改。
        因此我们创建一个新的IMU消息来进行修改和发布。
        """
        # 1. 创建一个新的Imu消息
        processed_imu = Imu()
        processed_imu.header = msg.header
        processed_imu.header.frame_id = self.base_link_frame

        # 2. 复制其他数据
        processed_imu.orientation = msg.orientation
        processed_imu.angular_velocity = msg.angular_velocity
        processed_imu.linear_acceleration = msg.linear_acceleration

        # 3. 将协方差元组转为列表，修改后赋给新消息
        # robot_localization要求协方差不能为0
        orientation_cov = list(msg.orientation_covariance)
        orientation_cov[0] = 0.001
        orientation_cov[4] = 0.001
        orientation_cov[8] = 0.001
        processed_imu.orientation_covariance = orientation_cov

        angular_velocity_cov = list(msg.angular_velocity_covariance)
        angular_velocity_cov[0] = 0.001
        angular_velocity_cov[4] = 0.001
        angular_velocity_cov[8] = 0.001
        processed_imu.angular_velocity_covariance = angular_velocity_cov

        linear_acceleration_cov = list(msg.linear_acceleration_covariance)
        linear_acceleration_cov[0] = 0.001
        linear_acceleration_cov[4] = 0.001
        linear_acceleration_cov[8] = 0.001
        processed_imu.linear_acceleration_covariance = linear_acceleration_cov
        
        # 4. 发布修改后的新消息
        self.imu_pub.publish(processed_imu)

    def dvl_callback(self, msg):
        """
        处理DVL消息。
        DVL提供了相对于水流的速度，我们需要将其转换为TwistWithCovarianceStamped消息。
        """
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = msg.header
        # 关键修正：必须使用DVL传感器的真实坐标系"dvl_link"，而不是"base_link"
        twist_msg.header.frame_id = self.dvl_link_frame
        twist_msg.twist.twist.linear = msg.velocity
        
        # 设定协方差。DVL的速度测量通常比较准，我们给一个较小的协方差
        twist_msg.twist.covariance[0] = 0.01   # X速度方差
        twist_msg.twist.covariance[7] = 0.01   # Y速度方差
        twist_msg.twist.covariance[14] = 0.01  # Z速度方差
        self.dvl_pub.publish(twist_msg)

    def depth_callback(self, msg):
        """
        处理深度计（压力传感器）消息。
        压力传感器给的是压强，这里我们做一个简化的转换，假设它直接给出了深度值，
        并将其转换为PoseWithCovarianceStamped消息。
        注意：在实际应用中，需要根据水密度进行精确转换。
        """
        # 1. 仿真器发布的fluid_pressure字段单位是kPa
        pressure_kpa = msg.fluid_pressure
        
        # 2. 将流体压强(kPa)根据公式转换为深度(m)
        # 注意处理边界情况, 避免除以零或负压
        if pressure_kpa >= ATM_PRESSURE_KPA:
            depth_m = (pressure_kpa - ATM_PRESSURE_KPA) / KPA_PER_METER
        else:
            depth_m = 0.0

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        # 2. EKF的world_frame是odom, 所以在odom坐标系下发布深度
        pose_msg.header.frame_id = self.odom_frame

        # 3. ROS ENU坐标系中Z轴朝上, 深度为负
        pose_msg.pose.pose.position.z = -depth_m

        # 深度传感器不提供姿态信息
        pose_msg.pose.pose.orientation.w = 1.0

        # Z轴的协方差设置得很小, XY和姿态的协方差非常大
        pose_msg.pose.covariance = np.diag([
            9999.0, 9999.0, 0.01,   # X, Y, Z
            9999.0, 9999.0, 9999.0  # Roll, Pitch, Yaw
        ]).flatten().tolist()
        self.depth_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('sensor_processor')
    try:
        node = SensorProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("传感器处理器节点已关闭。") 