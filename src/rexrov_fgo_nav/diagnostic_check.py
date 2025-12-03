#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Vector3Stamped
import tf.transformations as tft
import math

def get_rpy(quat):
    return tft.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

class Diagnostic:
    def __init__(self):
        rospy.init_node('fgo_diagnostic', anonymous=True)
        
        self.gt_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.gt_cb)
        self.dvl_sub = rospy.Subscriber('/rexrov/dvl', DVL, self.dvl_cb)
        self.imu_sub = rospy.Subscriber('/rexrov/imu', Imu, self.imu_cb)
        self.dvl_debug_sub = rospy.Subscriber('/rexrov/dvl_body_velocity', Vector3Stamped, self.dvl_debug_cb)
        
        self.latest_gt = None
        
    def gt_cb(self, msg):
        self.latest_gt = msg
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        r, p, y = get_rpy(ori)
        vel = msg.twist.twist.linear
        
        print(f"--- GT ---")
        print(f"Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        print(f"RPY (deg): ({math.degrees(r):.1f}, {math.degrees(p):.1f}, {math.degrees(y):.1f})")
        print(f"Vel World: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
        
        # Calculate Body Velocity from GT
        # R_wb * v_body = v_world  => v_body = R_wb^T * v_world
        R = tft.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])[:3, :3]
        import numpy as np
        v_w = np.array([vel.x, vel.y, vel.z])
        v_b = np.dot(R.T, v_w)
        print(f"Vel Body (Calculated): ({v_b[0]:.2f}, {v_b[1]:.2f}, {v_b[2]:.2f})")
        print("")

    def dvl_cb(self, msg):
        print(f"--- DVL RAW (Frame: {msg.header.frame_id}) ---")
        print(f"Vel: ({msg.velocity.x:.2f}, {msg.velocity.y:.2f}, {msg.velocity.z:.2f})")
        print("")

    def dvl_debug_cb(self, msg):
        print(f"--- DVL PROCESSED (Frame: {msg.header.frame_id}) ---")
        print(f"Vel: ({msg.vector.x:.2f}, {msg.vector.y:.2f}, {msg.vector.z:.2f})")
        
        if self.latest_gt:
             # Compare with GT Body Velocity
             # (Re-calculate for sync roughly)
             pass
        print("")

    def imu_cb(self, msg):
        r, p, y = get_rpy(msg.orientation)
        print(f"--- IMU ---")
        print(f"RPY (deg): ({math.degrees(r):.1f}, {math.degrees(p):.1f}, {math.degrees(y):.1f})")
        print("")

if __name__ == '__main__':
    d = Diagnostic()
    rospy.spin()
