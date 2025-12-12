# UUV ESKF Navigation - 项目开发指南

## 🌊 项目概述

**uuv_eskf_nav** 是一个基于误差状态卡尔曼滤波(Error-State Kalman Filter, ESKF)的水下载具(UUV)导航系统。该项目专注于实现高精度、实时的多传感器融合导航，支持IMU、DVL、深度传感器和地形匹配等多种传感器输入。

### 核心特性

- **🔬 高精度算法**: 15维误差状态向量精确建模，支持IMU偏差在线估计
- **🔧 通用设计**: 支持任意UUV仿真载具，只需修改robot_name参数
- **📡 多传感器融合**: 智能融合IMU、DVL、深度传感器数据
- **⚡ 实时性能**: 30Hz+ 导航更新频率
- **📊 在线评估**: 实时导航精度评估和误差分析
- **🗺️ 地形匹配**: GPU加速的粒子滤波地形匹配功能

## 🏗️ 系统架构

### 核心组件

```
src/
├── eskf_core.cpp           # ESKF核心算法实现
├── sensor_manager.cpp      # 传感器数据管理
├── eskf_navigation_node.cpp # 主导航节点
└── ...

include/
├── eskf_types.h           # 数据结构定义
├── eskf_core.h            # ESKF算法接口
└── sensor_manager.h       # 传感器管理接口

config/
├── eskf_params.yaml       # 核心配置文件

launch/
├── eskf_navigation.launch # 主启动文件

scripts/                   # Python脚本和GPU加速功能
├── terrain_matching_node_gpu.py  # GPU加速地形匹配
├── enhanced_navigation_evaluator.py  # 导航评估器
└── ...
```

### ESKF算法核心

**状态向量定义**:
- **主状态** (16维): 位置(3) + 速度(3) + 四元数(4) + 陀螺偏差(3) + 加速度偏差(3)
- **误差状态** (15维): δp(3) + δv(3) + δθ(3) + δbg(3) + δba(3)

**算法流程**:
1. **预测步骤**: IMU数据驱动的主状态传播 + 误差协方差预测
2. **更新步骤**: DVL/深度/地形测量的误差状态更新
3. **注入步骤**: 误差状态→主状态，误差状态重置

## 🚀 快速开始

### 环境准备

确保安装以下依赖:
```bash
# ROS Noetic + 基础工具
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-robot-localization
sudo apt install libeigen3-dev

# UUV Simulator (如果还没安装)
sudo apt install ros-noetic-uuv-simulator
```

### 编译系统

```bash
cd ~/catkin_ws
catkin_make  # 或使用 catkin build
source devel/setup.bash
```

### 启动ESKF导航系统

```bash
# 方法1: 使用默认配置 (ECA A9)
roslaunch uuv_eskf_nav eskf_navigation.launch

# 方法2: 指定载具
roslaunch uuv_eskf_nav eskf_navigation.launch robot_name:=rexrov2

# 方法3: 启用可视化和评估
roslaunch uuv_eskf_nav eskf_navigation.launch enable_rviz:=true enable_evaluator:=true
```

### 启用地形匹配功能

```bash
# 启动GPU加速地形匹配
roslaunch uuv_eskf_nav start_terrain_matching_gpu.launch
```

## ⚙️ 配置说明

### 核心配置文件: `config/eskf_params.yaml`

```yaml
# 机器人配置
robot_name: "eca_a9"              # 载具名称
world_frame: "odom"               # 世界坐标系名称
base_link_frame: "eca_a9/base_link"  # 机器人基座坐标系名称

# IMU噪声参数 (精确匹配URDF传感器规格)
imu:
  gyro_noise_std: 2.9088e-7       # 陀螺噪声 [rad/s]
  accel_noise_std: 1e-5           # 加速度噪声 [m/s²]
  gyro_bias_std: 8.1e-11          # 陀螺偏差游走 [rad/s]
  accel_bias_std: 1.6e-4          # 加速度偏差游走 [m/s²]

# DVL参数
dvl:
  noise_std: 1.3                  # DVL速度噪声 [m/s]

# 深度传感器参数
depth:
  noise_std: 0.5                  # 深度测量噪声 [m]

# 航向量测参数
heading:
  noise_std: 50000                # 航向角噪声 [rad]，禁用设为大值

# 初始状态
initial_state:
  position: [0.3183, 1.9111, 0.0] # 初始位置 [纬度, 经度, 高度]
  velocity: [0.0, 0.0, 0.0]       # 初始速度 [vx, vy, vz]
  attitude: [0.0, 0.0, 0.0]       # 初始姿态 [roll, pitch, yaw]

# 算法参数
algorithm_params:
  enable_earth_rotation: true     # 是否启用地球自转补偿
  mission_latitude_deg: 18.25     # 任务区域纬度 [度]
  enable_coning_correction: true  # 启用圆锥误差补偿
  enable_sculling_correction: true # 启用划桨效应补偿

# 传感器开关
sensors:
  enable_dvl: true
  enable_heading: false
  enable_terrain_nav: true
```

### 参数调优指南

| 参数类型 | 调大 → 效果 | 调小 → 效果 | 推荐范围 |
|---------|------------|------------|----------|
| **IMU噪声** | 更信任模型预测 | 更信任IMU测量 | 1e-7 - 1e-3 |
| **DVL噪声** | 更信任ESKF预测 | 更信任DVL测量 | 0.01 - 0.5 |
| **初始不确定性** | 收敛更慢但稳定 | 收敛快但可能发散 | 见配置文件 |

## 📊 性能监控

系统提供三层性能监控:

### 1. 实时监控 (1Hz)
```
=== ESKF导航系统实时性能评估 ===
位置误差:     0.0234 米
速度误差:     0.0156 米/秒  
姿态误差:     1.234 度
```

### 2. 长期统计 (30秒)
```
=== ESKF导航系统长期性能统计 ===
评估样本数:           1250
平均位置误差:         0.0189 米
最大位置误差:         0.0456 米
>>> 位置估计精度: 优秀 (< 0.1m)
```

### 3. 系统状态监控
- 传感器数据频率统计
- 协方差矩阵监控
- 数值稳定性检查

## 🔧 开发与调试

### 编译系统

项目使用CMake构建系统，支持C++17标准：

```cmake
add_compile_options(-std=c++17 -O3)
```

### 代码架构

- **eskf_core.cpp**: 实现ESKF核心算法，包括预测、更新、注入步骤
- **sensor_manager.cpp**: 负责处理IMU、DVL、深度传感器数据，包括坐标变换和杠杆臂补偿
- **eskf_navigation_node.cpp**: 主ROS节点，集成了ESKF算法和传感器管理

### 调试技巧

```bash
# 1. 检查传感器话题
rostopic list | grep your_robot_name
rostopic hz /your_robot_name/imu

# 2. 查看ESKF节点信息  
rosnode info /eskf_navigation

# 3. 监控协方差变化
rostopic echo /eskf/odometry/filtered/pose/covariance

# 4. 对比地面真值
rostopic echo /your_robot_name/pose_gt
```

### 故障排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|----------|
| **"等待传感器数据"** | 载具仿真未启动 | 检查gazebo中载具是否正常运行 |
| **位置误差很大** | 初始状态设置错误 | 调整config/eskf_params.yaml中initial_state |
| **系统发散** | 噪声参数不合理 | 增大过程噪声，减小观测噪声 |
| **编译失败** | 依赖库缺失 | 安装Eigen3, 检查ROS环境 |

## 🧪 GPU加速地形匹配

项目包含GPU加速的地形匹配功能，使用粒子滤波算法进行精确定位：

- **terrain_matching_node_gpu.py**: GPU加速的粒子滤波器
- **terrain_map_server_gpu.py**: GPU加速的地形高程查询
- 支持MBES(多波束声纳)数据输入，实现高精度海底地形匹配

## 📚 技术细节

### 算法理论基础

本系统基于Joan Solà的经典论文实现:
> *"Quaternion kinematics for the error-state Kalman filter"*

核心创新点:
1. **误差状态建模**: 避免传统EKF的线性化误差积累
2. **四元数误差**: 使用旋转向量表示姿态误差，避免奇异性
3. **IMU偏差估计**: 在线估计和补偿IMU系统误差
4. **改进机械编排**: 包含圆锥误差和划桨效应补偿

### 传感器处理

- **IMU**: 使用改进的机械编排算法，包含圆锥误差和划桨效应补偿
- **DVL**: 实现坐标变换和杠杆臂补偿，从dvl_link坐标系转换到base_link坐标系
- **深度传感器**: 压力到深度转换，包含动态不确定性调整
- **地形匹配**: GPU加速的粒子滤波器，使用MBES数据进行海底地形匹配

## 🤝 贡献指南

### 开发环境
```bash
# 克隆开发分支
git checkout -b feature/your-feature

# 代码风格
# - C++17标准
# - Google Style Guide
# - 详细注释

# 提交PR前请测试
catkin_make  
```

### 扩展支持新载具

1. 确保载具发布标准传感器话题
2. 在`config/`中添加载具专用配置
3. 更新README的载具支持列表
4. 提交测试结果

## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件