#!/usr/bin/env python3
import rospy
import tf2_ros
from uuv_sensor_ros_plugins_msgs.msg import DVL

def dvl_callback(msg):
    print(f"DVL Frame ID: '{msg.header.frame_id}'")
    rospy.signal_shutdown("Got DVL message")

if __name__ == "__main__":
    rospy.init_node("check_tf_debug")
    
    # 1. Check DVL Topic
    print("Waiting for DVL message...")
    rospy.Subscriber("/rexrov/dvl", DVL, dvl_callback)
    
    # 2. Check TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    try:
        print("Checking TF: rexrov/base_link -> rexrov/dvl_link")
        trans = tf_buffer.lookup_transform("rexrov/base_link", "rexrov/dvl_link", rospy.Time(0), rospy.Duration(5.0))
        print("TF Found!")
        print(trans)
    except Exception as e:
        print(f"TF Lookup Failed: {e}")

    rospy.spin()
