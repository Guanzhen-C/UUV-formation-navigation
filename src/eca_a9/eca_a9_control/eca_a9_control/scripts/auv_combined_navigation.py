#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
深海AUV组合导航系统
使用EKF融合IMU、DVL、压力传感器和GPS数据，替代Pose 3D传感器
"""

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from scipy.linalg import block_diag
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Header


class AUVCombinedNavigation:
    """深海AUV组合导航系统"""
    
    def __init__(self):
        rospy.init_node('auv_combined_navigation', anonymous=True)
        
        # 获取参数
        self.namespace = rospy.get_param('~namespace', 'eca_a9')
        self.rate = rospy.get_param('~rate', 20.0)
        
        # 初始化EKF
        self.init_ekf()
        
        # 初始化传感器数据
        self.imu_data = None
        self.dvl_data = None
        self.pressure_data = None
        self.gps_data = None
        
        # 初始化时间
        self.last_time = None
        self.dt = 0.0
        
        # 设置发布者和订阅者
        self.setup_publishers()
        self.setup_subscribers()
        
        # 初始化TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("AUV组合导航系统初始化完成")
    
    def init_ekf(self):
        """初始化扩展卡尔曼滤波器"""
        # 状态向量: [x, y, z, vx, vy, vz, roll, pitch, yaw, bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz]
        # 维度: 15
        self.state_dim = 15
        self.measurement_dim = 9  # IMU + DVL + Pressure + GPS
        
        # 初始化EKF
        self.ekf = ExtendedKalmanFilter(dim_x=self.state_dim, dim_z=self.measurement_dim)
        
        # 初始化状态向量
        self.ekf.x = np.zeros(self.state_dim)
        
        # 初始化状态协方差矩阵
        self.ekf.P = np.eye(self.state_dim) * 0.1
        
        # 过程噪声协方差矩阵
        self.ekf.Q = np.eye(self.state_dim) * 0.01
        
        # 测量噪声协方差矩阵
        self.ekf.R = np.eye(self.measurement_dim) * 0.1
        
        # 设置测量噪声
        # IMU噪声
        self.ekf.R[0:3, 0:3] = np.eye(3) * 0.01  # 加速度计
        self.ekf.R[3:6, 3:6] = np.eye(3) * 0.001  # 陀螺仪
        
        # DVL噪声
        self.ekf.R[6:9, 6:9] = np.eye(3) * 0.05  # 速度
        
        # 压力传感器噪声
        self.ekf.R[9:12, 9:12] = np.eye(3) * 0.1  # 深度
        
        # GPS噪声
        self.ekf.R[12:15, 12:15] = np.eye(3) * 1.0  # 位置
        
        rospy.loginfo("EKF初始化完成")
    
    def setup_publishers(self):
        """设置发布者"""
        # 发布融合后的位姿信息（替代pose_gt）
        self.pose_pub = rospy.Publisher(
            f'/{self.namespace}/pose_gt', 
            nav_msgs.msg.Odometry, 
            queue_size=1
        )
        
        # 发布导航状态
        self.nav_status_pub = rospy.Publisher(
            f'/{self.namespace}/nav_status', 
            geometry_msgs.msg.PoseStamped, 
            queue_size=1
        )
        
        # 发布传感器状态
        self.sensor_status_pub = rospy.Publisher(
            f'/{self.namespace}/sensor_status', 
            geometry_msgs.msg.PoseStamped, 
            queue_size=1
        )
    
    def setup_subscribers(self):
        """设置订阅者"""
        # 订阅IMU数据
        rospy.Subscriber(
            f'/{self.namespace}/imu', 
            sensor_msgs.msg.Imu, 
            self.imu_callback
        )
        
        # 订阅DVL数据
        rospy.Subscriber(
            f'/{self.namespace}/dvl', 
            nav_msgs.msg.Odometry, 
            self.dvl_callback
        )
        
        # 订阅压力传感器数据
        rospy.Subscriber(
            f'/{self.namespace}/pressure', 
            sensor_msgs.msg.FluidPressure, 
            self.pressure_callback
        )
        
        # 订阅GPS数据
        rospy.Subscriber(
            f'/{self.namespace}/gps', 
            sensor_msgs.msg.NavSatFix, 
            self.gps_callback
        )
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.imu_data = msg
        self.process_sensor_data()
    
    def dvl_callback(self, msg):
        """DVL数据回调"""
        self.dvl_data = msg
        self.process_sensor_data()
    
    def pressure_callback(self, msg):
        """压力传感器数据回调"""
        self.pressure_data = msg
        self.process_sensor_data()
    
    def gps_callback(self, msg):
        """GPS数据回调"""
        self.gps_data = msg
        self.process_sensor_data()
    
    def process_sensor_data(self):
        """处理传感器数据并执行EKF更新"""
        current_time = rospy.Time.now()
        
        # 计算时间间隔
        if self.last_time is not None:
            self.dt = (current_time - self.last_time).to_sec()
        else:
            self.dt = 0.05  # 默认20Hz
        
        self.last_time = current_time
        
        # 检查是否有足够的传感器数据
        if not self.has_sufficient_data():
            return
        
        # 预测步骤
        self.predict()
        
        # 更新步骤
        self.update()
        
        # 发布融合结果
        self.publish_fused_pose()
    
    def has_sufficient_data(self):
        """检查是否有足够的传感器数据"""
        # 至少需要IMU数据
        if self.imu_data is None:
            return False
        
        # 检查其他传感器数据
        sensors_available = {
            'IMU': self.imu_data is not None,
            'DVL': self.dvl_data is not None,
            'Pressure': self.pressure_data is not None,
            'GPS': self.gps_data is not None
        }
        
        # 发布传感器状态
        self.publish_sensor_status(sensors_available)
        
        return True
    
    def predict(self):
        """EKF预测步骤"""
        # 获取IMU数据
        accel = np.array([
            self.imu_data.linear_acceleration.x,
            self.imu_data.linear_acceleration.y,
            self.imu_data.linear_acceleration.z
        ])
        
        gyro = np.array([
            self.imu_data.angular_velocity.x,
            self.imu_data.angular_velocity.y,
            self.imu_data.angular_velocity.z
        ])
        
        # 状态转移函数
        self.ekf.predict(u=np.concatenate([accel, gyro]))
        
        # 更新过程噪声
        self.ekf.Q = self.compute_process_noise()
    
    def update(self):
        """EKF更新步骤"""
        # 构建测量向量
        z = self.build_measurement_vector()
        
        # 测量函数
        H = self.compute_measurement_jacobian()
        
        # 更新
        self.ekf.update(z, HJacobian=H, Hx=self.measurement_function)
    
    def build_measurement_vector(self):
        """构建测量向量"""
        z = np.zeros(self.measurement_dim)
        
        # IMU测量 (加速度和角速度)
        if self.imu_data is not None:
            accel = np.array([
                self.imu_data.linear_acceleration.x,
                self.imu_data.linear_acceleration.y,
                self.imu_data.linear_acceleration.z
            ])
            gyro = np.array([
                self.imu_data.angular_velocity.x,
                self.imu_data.angular_velocity.y,
                self.imu_data.angular_velocity.z
            ])
            z[0:3] = accel
            z[3:6] = gyro
        
        # DVL测量 (速度)
        if self.dvl_data is not None:
            vel = np.array([
                self.dvl_data.twist.twist.linear.x,
                self.dvl_data.twist.twist.linear.y,
                self.dvl_data.twist.twist.linear.z
            ])
            z[6:9] = vel
        
        # 压力传感器测量 (深度)
        if self.pressure_data is not None:
            # 将压力转换为深度 (简化计算)
            depth = self.pressure_to_depth(self.pressure_data.fluid_pressure)
            z[9:12] = np.array([0, 0, depth])
        
        # GPS测量 (位置)
        if self.gps_data is not None:
            # 将GPS坐标转换为局部坐标 (简化)
            pos = self.gps_to_local(self.gps_data.latitude, self.gps_data.longitude)
            z[12:15] = pos
        
        return z
    
    def pressure_to_depth(self, pressure):
        """将压力转换为深度"""
        # 简化计算: 1 bar ≈ 10米水深
        atmospheric_pressure = 1.01325  # bar
        depth = (pressure - atmospheric_pressure) * 10.0
        return depth
    
    def gps_to_local(self, lat, lon):
        """将GPS坐标转换为局部坐标"""
        # 简化转换，实际应用中需要更复杂的坐标转换
        # 这里假设有一个参考点
        ref_lat = 0.0  # 参考纬度
        ref_lon = 0.0  # 参考经度
        
        # 简单的线性近似
        x = (lon - ref_lon) * 111320.0  # 米
        y = (lat - ref_lat) * 111320.0  # 米
        
        return np.array([x, y, 0])
    
    def compute_process_noise(self):
        """计算过程噪声协方差矩阵"""
        # 基于时间间隔调整过程噪声
        dt = self.dt
        
        # 位置噪声
        pos_noise = 0.1 * dt
        
        # 速度噪声
        vel_noise = 0.05 * dt
        
        # 姿态噪声
        att_noise = 0.01 * dt
        
        # 偏置噪声
        bias_noise = 0.001 * dt
        
        Q = np.eye(self.state_dim)
        Q[0:3, 0:3] *= pos_noise      # 位置
        Q[3:6, 3:6] *= vel_noise      # 速度
        Q[6:9, 6:9] *= att_noise      # 姿态
        Q[9:12, 9:12] *= bias_noise   # 加速度计偏置
        Q[12:15, 12:15] *= bias_noise # 陀螺仪偏置
        
        return Q
    
    def compute_measurement_jacobian(self):
        """计算测量雅可比矩阵"""
        H = np.zeros((self.measurement_dim, self.state_dim))
        
        # IMU测量雅可比
        H[0:3, 9:12] = np.eye(3)  # 加速度计偏置
        H[3:6, 12:15] = np.eye(3) # 陀螺仪偏置
        
        # DVL测量雅可比
        H[6:9, 3:6] = np.eye(3)   # 速度
        
        # 压力传感器测量雅可比
        H[9:12, 2] = np.array([0, 0, 1])  # 深度
        
        # GPS测量雅可比
        H[12:15, 0:3] = np.eye(3)  # 位置
        
        return H
    
    def measurement_function(self, x):
        """测量函数"""
        h = np.zeros(self.measurement_dim)
        
        # IMU测量
        h[0:3] = x[9:12]  # 加速度计偏置
        h[3:6] = x[12:15] # 陀螺仪偏置
        
        # DVL测量
        h[6:9] = x[3:6]   # 速度
        
        # 压力传感器测量
        h[9:12] = np.array([0, 0, x[2]])  # 深度
        
        # GPS测量
        h[12:15] = x[0:3]  # 位置
        
        return h
    
    def publish_fused_pose(self):
        """发布融合后的位姿信息"""
        # 创建Odometry消息
        odom_msg = nav_msgs.msg.Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = f"{self.namespace}/base_link"
        
        # 设置位置
        odom_msg.pose.pose.position.x = self.ekf.x[0]
        odom_msg.pose.pose.position.y = self.ekf.x[1]
        odom_msg.pose.pose.position.z = self.ekf.x[2]
        
        # 设置姿态 (欧拉角转四元数)
        roll, pitch, yaw = self.ekf.x[6], self.ekf.x[7], self.ekf.x[8]
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # 设置速度
        odom_msg.twist.twist.linear.x = self.ekf.x[3]
        odom_msg.twist.twist.linear.y = self.ekf.x[4]
        odom_msg.twist.twist.linear.z = self.ekf.x[5]
        odom_msg.twist.twist.angular.x = self.ekf.x[6]
        odom_msg.twist.twist.angular.y = self.ekf.x[7]
        odom_msg.twist.twist.angular.z = self.ekf.x[8]
        
        # 设置协方差矩阵
        pose_cov = np.zeros(36)
        twist_cov = np.zeros(36)
        
        # 位置协方差
        pose_cov[0] = self.ekf.P[0, 0]  # x
        pose_cov[7] = self.ekf.P[1, 1]  # y
        pose_cov[14] = self.ekf.P[2, 2] # z
        
        # 姿态协方差
        pose_cov[21] = self.ekf.P[6, 6]  # roll
        pose_cov[28] = self.ekf.P[7, 7]  # pitch
        pose_cov[35] = self.ekf.P[8, 8]  # yaw
        
        # 速度协方差
        twist_cov[0] = self.ekf.P[3, 3]  # vx
        twist_cov[7] = self.ekf.P[4, 4]  # vy
        twist_cov[14] = self.ekf.P[5, 5] # vz
        
        odom_msg.pose.covariance = pose_cov.tolist()
        odom_msg.twist.covariance = twist_cov.tolist()
        
        # 发布消息
        self.pose_pub.publish(odom_msg)
        
        # 发布导航状态
        nav_msg = geometry_msgs.msg.PoseStamped()
        nav_msg.header = odom_msg.header
        nav_msg.pose = odom_msg.pose.pose
        self.nav_status_pub.publish(nav_msg)
    
    def publish_sensor_status(self, sensors_available):
        """发布传感器状态"""
        status_msg = geometry_msgs.msg.PoseStamped()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.header.frame_id = "world"
        
        # 使用位置字段传递传感器状态信息
        status_msg.pose.position.x = float(sensors_available['IMU'])
        status_msg.pose.position.y = float(sensors_available['DVL'])
        status_msg.pose.position.z = float(sensors_available['Pressure'])
        
        # 使用姿态字段传递GPS状态
        status_msg.pose.orientation.w = float(sensors_available['GPS'])
        
        self.sensor_status_pub.publish(status_msg)
    
    def run(self):
        """运行组合导航系统"""
        rate = rospy.Rate(self.rate)
        
        rospy.loginfo("开始运行AUV组合导航系统")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        
        rospy.loginfo("AUV组合导航系统已停止")


if __name__ == '__main__':
    try:
        navigation = AUVCombinedNavigation()
        navigation.run()
    except rospy.ROSInterruptException:
        pass 