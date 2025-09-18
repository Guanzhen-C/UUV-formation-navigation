#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESKF导航系统启动信息显示脚本

在系统启动时显示配置信息和使用提示
"""

import rospy
import os

def main():
    rospy.init_node('startup_message', anonymous=True)
    
    # 获取参数 - 首先尝试从私有命名空间读取，然后从全局命名空间读取
    robot_name = rospy.get_param('~robot_name', rospy.get_param('robot_name', ''))
    if not robot_name:
        rospy.logfatal("必须在配置文件中定义robot_name参数!")
        rospy.signal_shutdown("缺少robot_name参数")
        return
    config_file = rospy.get_param('~config_file', 'unknown')
    
    # 显示启动信息
    print("\n" + "=" * 80)
    print("🌊 ESKF水下导航系统启动成功 🌊")
    print("=" * 80)
    print("系统信息:")
    print(f"  载具名称:     {robot_name}")
    print(f"  配置文件:     {os.path.basename(config_file)}")
    print(f"  算法类型:     误差状态卡尔曼滤波 (Error-State Kalman Filter)")
    print(f"  状态维度:     15维误差状态向量")
    print()
    print("输出话题:")
    print("  /eskf/odometry/filtered    - 融合导航结果")
    print("  /eskf/pose                 - 位姿估计")
    print()
    print("传感器输入:")
    print(f"  /{robot_name}/imu          - 惯性测量单元")
    print(f"  /{robot_name}/dvl          - 多普勒测速仪")
    print(f"  /{robot_name}/pressure     - 深度传感器")
    print()
    print("系统特点:")
    print("  ✓ 15维误差状态精确建模")
    print("  ✓ IMU偏差在线估计")
    print("  ✓ 多传感器融合更新")
    print("  ✓ 实时性能监控")
    print("  ✓ 即插即用载具适配")
    print()
    print("使用提示:")
    print("  - 确保载具仿真正在运行")
    print("  - 检查传感器数据是否正常发布") 
    print("  - 查看 /eskf/odometry/filtered 获取导航结果")
    print("  - 使用 Ctrl+C 安全停止系统")
    print("=" * 80)
    print("🚀 系统已就绪，开始导航！")
    print("=" * 80 + "\n")
    
    # 等待一小段时间后退出（让信息显示完整）
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except:
        pass
