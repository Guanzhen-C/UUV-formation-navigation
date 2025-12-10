#!/usr/bin/env python3
import rospy
import numpy as np
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Vector3

def get_lawnmower_waypoints():
    waypoints = []
    
    # Configuration
    start_x = -400.0
    end_x = 400.0
    start_y = -400.0
    end_y = 400.0
    
    spacing = 100.0 # Wide spacing to cover more ground
    depth = -30.0
    speed = 1.5
    
    # Generate zigzag
    x = start_x
    direction = 1 # 1 for Up, -1 for Down
    
    current_time = 0.0
    
    while x <= end_x:
        # Start point of this leg
        p1_y = start_y if direction == 1 else end_y
        p2_y = end_y if direction == 1 else start_y
        
        # Leg 1: Vertical segment (Cross-contour mostly)
        dist = abs(p2_y - p1_y)
        duration = dist / speed
        
        # Add points for this leg (start and end)
        # In a real trajectory generator we would interpolate, 
        # but the controller might handle sparse waypoints if we send them as a list?
        # Actually, standard UUV trajectory msgs require dense points.
        # Let's generate dense points.
        
        for t in np.arange(0, duration, 1.0): # 1 second resolution
            ratio = t / duration
            cur_y = p1_y + (p2_y - p1_y) * ratio
            
            p = TrajectoryPoint()
            p.header.stamp = rospy.Time.now() + rospy.Duration(current_time + t)
            p.pose.position.x = x
            p.pose.position.y = cur_y
            p.pose.position.z = depth
            
            # Orientation (Yaw) - Facing North or South
            yaw = np.pi/2 if direction == 1 else -np.pi/2
            # Simplified quaternion (assuming flat pitch/roll)
            p.pose.orientation = Quaternion(0, 0, np.sin(yaw/2), np.cos(yaw/2))
            
            p.velocity.linear.x = speed # Body frame forward
            
            waypoints.append(p)
            
        current_time += duration
        
        # Leg 2: Horizontal segment (Transition to next line)
        if x + spacing <= end_x:
            next_x = x + spacing
            dist = spacing
            duration = dist / speed
            
            for t in np.arange(0, duration, 1.0):
                ratio = t / duration
                cur_x = x + (next_x - x) * ratio
                
                p = TrajectoryPoint()
                p.header.stamp = rospy.Time.now() + rospy.Duration(current_time + t)
                p.pose.position.x = cur_x
                p.pose.position.y = p2_y
                p.pose.position.z = depth
                
                yaw = 0.0 # Facing East
                p.pose.orientation = Quaternion(0, 0, np.sin(yaw/2), np.cos(yaw/2))
                p.velocity.linear.x = speed
                
                waypoints.append(p)
                
            current_time += duration
            x = next_x
        else:
            break
            
        direction *= -1
        
    return waypoints

def sender():
    rospy.init_node('lawnmower_path_publisher')
    
    # Check topic name - typically /<robot_name>/trajectory_control/input/trajectory
    # or /<robot_name>/reference/trajectory
    # Based on eca_a9 standard:
    topic = '/eca_a9/input/trajectory' # Common for geometric tracking controller
    
    pub = rospy.Publisher(topic, Trajectory, queue_size=1)
    
    rospy.loginfo(f"Waiting for subscribers on {topic}...")
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        rospy.sleep(0.1)
        
    rospy.loginfo("Generating Lawnmower Trajectory...")
    points = get_lawnmower_waypoints()
    
    traj = Trajectory()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = 'world'
    traj.points = points
    
    rospy.loginfo(f"Publishing trajectory with {len(points)} points...")
    pub.publish(traj)
    rospy.loginfo("Done!")

if __name__ == "__main__":
    sender()
