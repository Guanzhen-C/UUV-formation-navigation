#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def quaternion_to_yaw(qx, qy, qz, qw):
    # ZYX yaw from quaternion (w, x, y, z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class CompassFromGTNode(object):
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', 'eca_a9')
        default_gt = '/{}/pose_gt'.format(self.robot_name)
        default_out = '/{}/compass_yaw'.format(self.robot_name)
        self.gt_topic = rospy.get_param('~gt_topic', default_gt)
        self.out_topic = rospy.get_param('~compass_topic', default_out)
        self.rate_hz = float(rospy.get_param('~rate_hz', 50.0))  # 发布频率（Hz）

        self.pub = rospy.Publisher(self.out_topic, Float64, queue_size=50)
        rospy.Subscriber(self.gt_topic, Odometry, self.cb_gt, queue_size=50)

        self.last_yaw = None

        rospy.loginfo('CompassFromGT: robot=%s, gt_topic=%s, out_topic=%s',
                      self.robot_name, self.gt_topic, self.out_topic)
        rospy.loginfo('CompassFromGT: rate_hz=%.1f', self.rate_hz)

        # 定时发布，提升发布频率（对齐滤波器期望频率）
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate_hz)), self.on_timer)

    def cb_gt(self, msg):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        # normalize to [-pi, pi]
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi
        self.last_yaw = yaw

    def on_timer(self, event):
        if self.last_yaw is not None:
            self.pub.publish(Float64(self.last_yaw))


if __name__ == '__main__':
    rospy.init_node('compass_from_gt')
    CompassFromGTNode()
    rospy.spin()


