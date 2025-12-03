#!/usr/bin/env python3
import rospy
import tf2_ros
import sys

if __name__ == "__main__":
    rospy.init_node("check_sonar_tf_debug")
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Frames to check
    target_frame = "rexrov/base_link"
    source_frame = "forward_sonar_optical_frame" 
    intermediate_frame = "rexrov/forward_sonar_mbes_optical_frame"

    print(f"Waiting for TF tree to populate (checking {source_frame} -> {target_frame})...")
    rospy.sleep(2.0)

    # 1. Check if frames exist at all
    frames = tf_buffer.all_frames_as_yaml()
    if source_frame not in frames:
        print(f"[FAIL] Frame '{source_frame}' NOT FOUND in TF tree!")
    else:
        print(f"[OK] Frame '{source_frame}' found in TF tree.")

    if intermediate_frame not in frames:
        print(f"[FAIL] Frame '{intermediate_frame}' NOT FOUND in TF tree!")
    else:
        print(f"[OK] Frame '{intermediate_frame}' found in TF tree.")

    if target_frame not in frames:
        print(f"[FAIL] Frame '{target_frame}' NOT FOUND in TF tree!")
    else:
        print(f"[OK] Frame '{target_frame}' found in TF tree.")

    # 2. Try Lookup
    try:
        print(f"Attempting lookup: {target_frame} <-- {source_frame}")
        trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        print("[SUCCESS] Transform found!")
        print(trans)
    except Exception as e:
        print(f"[FAIL] Lookup failed: {e}")
        print("\n--- All Frames in TF Tree ---")
        print(frames)
