#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Vector3Stamped
import tf.transformations as tft
import math
import numpy as np

class SensorCheck:
    def __init__(self):
        rospy.init_node('sensor_check', anonymous=True)
        
        self.got_gt = False
        self.got_dvl = False
        self.got_dvl_proc = False
        self.got_imu = False

        rospy.Subscriber('/rexrov/pose_gt', Odometry, self.gt_cb)
        rospy.Subscriber('/rexrov/dvl', DVL, self.dvl_cb)
        rospy.Subscriber('/rexrov/dvl_body_velocity', Vector3Stamped, self.dvl_proc_cb)
        rospy.Subscriber('/rexrov/imu', Imu, self.imu_cb)
        
        print("Waiting for messages...")

    def gt_cb(self, msg):
        if self.got_gt: return
        self.got_gt = True
        
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        r, p, y = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        v_world = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        
        # R_wb (Body to World)
        R_wb = tft.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])[:3, :3]
        # v_body = R_wb^T * v_world
        v_body = np.dot(R_wb.T, v_world)
        
        print(f"\n[Ground Truth]")
        print(f"  Pos:   x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        print(f"  Yaw:   {math.degrees(y):.1f} deg")
        print(f"  V_wld: x={v_world[0]:.3f}, y={v_world[1]:.3f}, z={v_world[2]:.3f}")
        print(f"  V_bdy: x={v_body[0]:.3f} (Fwd), y={v_body[1]:.3f} (Left), z={v_body[2]:.3f} (Up)")

    def dvl_cb(self, msg):
        if self.got_dvl: return
        self.got_dvl = True
        print(f"\n[DVL Raw] Frame: {msg.header.frame_id}")
        print(f"  Vel:   x={msg.velocity.x:.3f}, y={msg.velocity.y:.3f}, z={msg.velocity.z:.3f}")

    def dvl_proc_cb(self, msg):
        if self.got_dvl_proc: return
        self.got_dvl_proc = True
        print(f"\n[DVL Processed (Body)] Frame: {msg.header.frame_id}")
        print(f"  Vel:   x={msg.vector.x:.3f}, y={msg.vector.y:.3f}, z={msg.vector.z:.3f}")

    def imu_cb(self, msg):
        if self.got_imu: return
        self.got_imu = True
        ori = msg.orientation
        r, p, y = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        print(f"\n[IMU]")
        print(f"  Yaw:   {math.degrees(y):.1f} deg")
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.got_gt and self.got_dvl and self.got_dvl_proc and self.got_imu:
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        c = SensorCheck()
        c.run()
    except rospy.ROSInterruptException:
        pass
