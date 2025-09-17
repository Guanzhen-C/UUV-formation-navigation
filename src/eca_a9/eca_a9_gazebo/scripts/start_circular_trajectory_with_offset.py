#!/usr/bin/env python
# 自定义圆形轨迹生成器，支持角度偏移
# 基于UUV Simulator的start_circular_trajectory.py修改

from __future__ import print_function
import rospy
import sys
from uuv_control_msgs.srv import InitCircularTrajectory
from numpy import pi
from geometry_msgs.msg import Point
from std_msgs.msg import Time


if __name__ == '__main__':
    print('Starting the custom circular trajectory creator with angle offset')
    rospy.init_node('start_circular_trajectory_with_offset')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    # If no start time is provided: start *now*.
    start_time = rospy.Time.now().to_sec()
    start_now = False
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
        if start_time < 0.0:
            print('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
    else:
        start_now = True

    param_labels = ['radius', 'center', 'n_points', 'heading_offset',
                    'angle_offset', 'duration', 'max_forward_speed']
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

    try:
        rospy.wait_for_service('start_circular_trajectory', timeout=20)
    except rospy.ROSException:
        print('Service not available! Closing node...')
        sys.exit(-1)

    try:
        traj_gen = rospy.ServiceProxy('start_circular_trajectory', InitCircularTrajectory)
    except rospy.ServiceException as e:
        print('Service call failed, error={}'.format(e))
        sys.exit(-1)

    print('Generating trajectory that starts at t={} s'.format(start_time))
    print('Using angle_offset={} degrees'.format(params['angle_offset']))

    success = traj_gen(Time(rospy.Time(start_time)),
                       start_now,
                       params['radius'],
                       Point(params['center'][0], params['center'][1], params['center'][2]),
                       False,
                       params['angle_offset'] * pi / 180,  # 使用自定义角度偏移
                       params['n_points'],
                       params['heading_offset'] * pi / 180,
                       params['max_forward_speed'],
                       params['duration'])

    if success:
        print('Trajectory successfully generated!')
    else:
        print('Failed')



