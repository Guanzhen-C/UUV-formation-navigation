#!/usr/bin/env python
# 生成相同轨迹路径的圆形轨迹生成器
# 所有AUV使用相同的轨迹路径，但通过不同的起始时间实现不同起始位置

from __future__ import print_function
import rospy
import sys
from uuv_control_msgs.srv import InitCircularTrajectory
from numpy import pi
from geometry_msgs.msg import Point
from std_msgs.msg import Time


if __name__ == '__main__':
    print('Starting the circular trajectory creator with same path')
    rospy.init_node('start_circular_trajectory_same_path')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    # 获取AUV名称，用于计算时间偏移
    uuv_name = rospy.get_namespace().replace('/', '')
    print('Creating trajectory for: {}'.format(uuv_name))

    # 基础起始时间
    base_start_time = rospy.Time.now().to_sec()
    start_now = False
    if rospy.has_param('~start_time'):
        base_start_time = rospy.get_param('~start_time')
        if base_start_time < 0.0:
            print('Negative start time, setting it to 0.0')
            base_start_time = 0.0
            start_now = True
    else:
        start_now = True

    # 所有AUV使用相同的起始时间，轨迹路径完全一致
    actual_start_time = base_start_time
    
    # 获取AUV编号用于日志
    auv_number = 1  # 默认值
    if 'eca_a9_' in uuv_name:
        try:
            auv_number = int(uuv_name.split('_')[-1])
        except:
            auv_number = 1
    
    print('AUV {}: Using same trajectory path, start time = {}'.format(
        auv_number, actual_start_time))
    print('AUV will find the closest point on the circular trajectory from its current position')

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

    print('Generating trajectory that starts at t={} s'.format(actual_start_time))
    print('All AUVs use the same trajectory path (angle_offset=0)')

    # 所有AUV使用相同的轨迹路径（angle_offset=0），但起始时间不同
    success = traj_gen(Time(rospy.Time(actual_start_time)),
                       start_now,
                       params['radius'],
                       Point(params['center'][0], params['center'][1], params['center'][2]),
                       False,
                       0.0,  # 固定angle_offset=0，所有AUV使用相同轨迹路径
                       params['n_points'],
                       params['heading_offset'] * pi / 180,
                       params['max_forward_speed'],
                       params['duration'])

    if success:
        print('Trajectory successfully generated!')
    else:
        print('Failed')
