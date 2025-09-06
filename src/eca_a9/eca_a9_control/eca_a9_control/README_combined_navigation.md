# 深海AUV组合导航系统

## 概述

本系统使用扩展卡尔曼滤波器（EKF）融合多个传感器数据，替代Pose 3D传感器，为深海AUV提供更可靠的导航解决方案。

## 系统架构

### 传感器融合
- **IMU（惯性测量单元）**：提供加速度和角速度信息
- **DVL（多普勒速度计程仪）**：提供相对于海底的速度信息
- **压力传感器**：提供深度信息
- **GPS**：提供水面位置信息（当AUV浮出水面时）

### 状态向量
```
[x, y, z, vx, vy, vz, roll, pitch, yaw, bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz]
```
- 位置：x, y, z
- 速度：vx, vy, vz
- 姿态：roll, pitch, yaw
- 传感器偏置：bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz

## 使用方法

### 1. 安装依赖
```bash
pip install filterpy scipy numpy
```

### 2. 启动组合导航系统
```bash
# 启动组合导航系统
roslaunch eca_a9_control start_combined_navigation.launch

# 启动使用组合导航的几何跟踪控制
roslaunch eca_a9_control start_geometric_tracking_with_fusion.launch
```

### 3. 参数配置

#### 传感器噪声参数
- `imu_noise_accel`：IMU加速度计噪声标准差（默认：0.01）
- `imu_noise_gyro`：IMU陀螺仪噪声标准差（默认：0.001）
- `dvl_noise_vel`：DVL速度噪声标准差（默认：0.05）
- `pressure_noise_depth`：压力传感器深度噪声标准差（默认：0.1）
- `gps_noise_pos`：GPS位置噪声标准差（默认：1.0）

#### 过程噪声参数
- `process_noise_pos`：位置过程噪声（默认：0.1）
- `process_noise_vel`：速度过程噪声（默认：0.05）
- `process_noise_att`：姿态过程噪声（默认：0.01）
- `process_noise_bias`：偏置过程噪声（默认：0.001）

## 话题说明

### 输入话题
- `/{namespace}/imu`：IMU数据（sensor_msgs/Imu）
- `/{namespace}/dvl`：DVL数据（nav_msgs/Odometry）
- `/{namespace}/pressure`：压力传感器数据（sensor_msgs/FluidPressure）
- `/{namespace}/gps`：GPS数据（sensor_msgs/NavSatFix）

### 输出话题
- `/{namespace}/pose_gt`：融合后的位姿信息（nav_msgs/Odometry）
- `/{namespace}/nav_status`：导航状态（geometry_msgs/PoseStamped）
- `/{namespace}/sensor_status`：传感器状态（geometry_msgs/PoseStamped）

## 算法特点

### 1. 自适应噪声估计
系统根据传感器数据的质量和可用性动态调整噪声参数。

### 2. 传感器故障检测
当某个传感器失效时，系统能够继续使用其他可用传感器进行导航。

### 3. 偏置估计
自动估计和补偿IMU的加速度计和陀螺仪偏置。

### 4. 深度约束
利用压力传感器提供的深度信息约束垂直位置估计。

## 性能优化

### 1. 调整噪声参数
根据实际传感器性能调整噪声参数：
```xml
<arg name="imu_noise_accel" value="0.005"/>  <!-- 更精确的IMU -->
<arg name="dvl_noise_vel" value="0.02"/>     <!-- 更精确的DVL -->
```

### 2. 调整更新频率
根据计算能力调整更新频率：
```xml
<arg name="rate" value="50.0"/>  <!-- 50Hz更新 -->
```

### 3. 传感器权重调整
在代码中修改测量噪声协方差矩阵来调整不同传感器的权重。

## 故障排除

### 1. 传感器数据缺失
- 检查传感器话题是否正常发布
- 确认传感器插件是否正确加载

### 2. 导航精度下降
- 调整噪声参数
- 检查传感器校准
- 增加更多传感器数据

### 3. 系统不稳定
- 降低更新频率
- 增加过程噪声
- 检查传感器数据质量

## 与原始Pose 3D的对比

| 特性 | Pose 3D | 组合导航系统 |
|------|---------|-------------|
| 数据来源 | Gazebo物理引擎 | 多传感器融合 |
| 精度 | 完美（Ground Truth） | 取决于传感器质量 |
| 鲁棒性 | 低（单一数据源） | 高（多传感器冗余） |
| 真实性 | 低（仿真环境） | 高（接近真实环境） |
| 计算复杂度 | 低 | 中等 |

## 扩展功能

### 1. 添加新传感器
在`build_measurement_vector()`方法中添加新传感器的测量处理。

### 2. 改进状态模型
修改状态转移函数以适应更复杂的运动模型。

### 3. 多模型滤波
实现交互式多模型（IMM）滤波器来处理不同的运动模式。

## 注意事项

1. **初始化**：系统需要一定时间收敛到稳定状态
2. **传感器同步**：确保所有传感器数据时间戳同步
3. **坐标系**：注意不同传感器使用不同的坐标系
4. **内存使用**：EKF计算可能消耗较多内存，注意监控系统资源

## 技术支持

如有问题，请检查：
1. ROS话题是否正常发布
2. 传感器插件是否正确加载
3. 参数配置是否合理
4. 系统日志中的错误信息 