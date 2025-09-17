# UUV导航融合系统

## 概述

这是一个通用的水下载具(UUV)导航融合系统，支持任意AUV/ROV的多传感器导航融合。系统使用扩展卡尔曼滤波器(EKF)融合IMU、DVL、深度传感器等数据，提供精确的水下导航定位。

## 核心特性

- **通用性强**：支持任意水下载具，只需设置机器人名称
- **传感器融合**：IMU + DVL + 压力传感器的智能融合
- **实时评估**：提供导航精度的实时评估和误差分析
- **即插即用**：无需修改代码，仅通过launch参数配置

## 系统架构

```
AUV传感器 → 传感器处理器 → EKF滤波器 → 融合导航结果
    ↓            ↓           ↓         ↓
/{robot}/imu  /imu/data   EKF融合   /odometry/filtered
/{robot}/dvl  /dvl/twist             (30Hz输出)
/{robot}/pressure /depth/pose
```

## 支持的载具

理论上支持所有UUV仿真器中的载具，包括但不限于：

- **ECA A9** - 自主水下载具
- **REXrov** - 研究级ROV  
- **LAUV** - 轻型自主水下载具
- **Desistek SAGA** - 遥控水下载具
- **自定义载具** - 只要遵循UUV仿真器的传感器话题约定

## 快速开始

### 1. 启动载具仿真
```bash
# 启动海洋环境
roslaunch uuv_gazebo_worlds ocean_waves.launch

# 启动你的载具 (以ECA A9为例)
roslaunch eca_a9_gazebo start_demo_auv_control.launch
```

### 2. 启动导航融合系统
```bash
# 方法1：使用默认配置 (ECA A9)
roslaunch uuv_nav_fusion uuv_nav.launch

# 方法2：指定载具名称
roslaunch uuv_nav_fusion uuv_nav.launch robot_name:=YOUR_ROBOT_NAME

# 示例：
roslaunch uuv_nav_fusion uuv_nav.launch robot_name:=rexrov
roslaunch uuv_nav_fusion uuv_nav.launch robot_name:=lauv
```

### 3. 查看结果
```bash
# 查看融合导航结果
rostopic echo /odometry/filtered

# 查看传感器处理状态
rosnode info /sensor_processor

# 监控导航精度 (会显示与地面真值的误差)
# 导航评估器会自动输出误差信息到终端
```

## 配置说明

### 核心参数

只有一个核心参数需要配置：

- `robot_name`: 载具名称，必须与仿真中的载具命名空间一致

### 高级参数 (通常不需要修改)

- `odom_frame`: 里程计坐标系 (默认: "odom")
- `base_link_frame`: 载具基准坐标系 (默认: "{robot_name}/base_link")
- `world_frame`: 世界坐标系 (默认: "odom")

## 切换不同载具

### 从ECA A9切换到REXrov
```bash
# 停止当前仿真
# 启动REXrov仿真
roslaunch rexrov2_gazebo start_demo_auv_control.launch

# 启动导航融合 (指定新的机器人名称)
roslaunch uuv_nav_fusion uuv_nav.launch robot_name:=rexrov2
```

### 从REXrov切换到LAUV
```bash
# 停止当前仿真  
# 启动LAUV仿真
roslaunch lauv_gazebo start_demo_auv_control.launch

# 启动导航融合
roslaunch uuv_nav_fusion uuv_nav.launch robot_name:=lauv
```

## 输出话题

- `/odometry/filtered`: 融合后的导航结果 (nav_msgs/Odometry)
- `/imu/data`: 处理后的IMU数据 (sensor_msgs/Imu)  
- `/dvl/twist`: 处理后的DVL速度数据 (geometry_msgs/TwistWithCovarianceStamped)
- `/depth/pose`: 处理后的深度数据 (geometry_msgs/PoseWithCovarianceStamped)

## 错误排查

### 1. "robot_name参数未设置"错误
**原因**：忘记在launch文件中设置robot_name参数  
**解决**：确保launch文件中包含 `<arg name="robot_name" default="YOUR_ROBOT"/>`

### 2. "正在等待传感器话题"警告
**原因**：载具仿真未启动或传感器话题名称不匹配  
**解决**：检查载具是否正常运行，确认话题名称格式为 `/{robot_name}/{sensor}`

### 3. EKF无输出
**原因**：传感器数据不足或EKF配置问题  
**解决**：检查传感器处理器是否正常工作，确认所有传感器数据都在发布

## 技术细节

- **EKF频率**: 30Hz
- **传感器处理**: 自动协方差调整和坐标系转换
- **误差评估**: 实时计算与地面真值的欧氏距离误差
- **坐标系**: 支持自动的多坐标系转换

## 贡献指南

要添加对新载具的支持，只需确保：
1. 载具发布标准的传感器话题：`/{robot_name}/imu`, `/{robot_name}/dvl`, `/{robot_name}/pressure`
2. 载具提供地面真值话题：`/{robot_name}/pose_gt`
3. 在launch文件中设置正确的`robot_name`参数

系统会自动适配新载具，无需修改任何代码！
