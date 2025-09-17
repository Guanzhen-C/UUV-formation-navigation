# 🧪 ESKF导航精度测试指南

## 📋 测试系统概述

我们基于你的`uuv_nav_fusion`测试方法，开发了**增强版ESKF导航测试系统**，相比原系统具有以下重要改进：

### 🎯 **原uuv_nav_fusion的测试方法**：
```bash
✅ AUV基于真实定位运行
✅ 收集传感器数据进行组合导航
✅ 输出导航结果与真值对比
❌ 只能在终端看到文字误差
❌ 无法在RViz中可视化轨迹对比
```

### 🚀 **我们的增强测试系统**：
```bash
✅ 保持原有的测试逻辑和精度分析
✅ RViz双轨迹可视化 (绿色真值 + 蓝色估计)
✅ 实时误差向量显示 (红色箭头)
✅ 历史误差热力图 (彩色球体)
✅ 详细统计报告 (均值/最大值/标准差)
✅ ROS话题发布统计数据供其他节点使用
```

## 🛠️ 测试环境准备

### 1️⃣ **启动AUV仿真环境**

首先在**终端1**中启动ECA A9仿真（或其他AUV）：

```bash
# 启动Gazebo仿真环境 + AUV模型
roslaunch eca_a9_gazebo start_pid_demo.launch

# 或者如果是其他AUV型号：
# roslaunch uuv_gazebo_worlds auv_demo.launch robot_name:=YOUR_AUV_NAME
```

等待仿真环境完全启动，确保：
- ✅ Gazebo界面正常显示
- ✅ AUV模型加载完成
- ✅ 传感器插件正常工作

### 2️⃣ **启动ESKF导航测试系统**

在**终端2**中启动我们的测试系统：

```bash
cd /home/qsk/catkin_ws_0905

# 启动完整测试系统 (包含RViz可视化)
roslaunch uuv_eskf_nav eskf_navigation_test.launch robot_name:=eca_a9

# 如果不需要RViz，可以关闭可视化
roslaunch uuv_eskf_nav eskf_navigation_test.launch robot_name:=eca_a9 start_rviz:=false
```

系统启动后你将看到：
- 🤖 改进ESKF导航节点运行
- 📊 增强版导航评估器启动
- 🖥️ RViz窗口自动打开（如果启用）

## 📊 测试结果观察

### 🖥️ **RViz可视化界面**

RViz中将显示以下内容：

#### **轨迹对比**：
- 🟢 **绿色路径**：`/navigation_test/ground_truth_path` (真值轨迹)
- 🔵 **蓝色路径**：`/navigation_test/estimated_path` (ESKF估计轨迹)

#### **误差可视化**：
- 🔴 **红色箭头**：`/navigation_test/current_error` (当前误差向量)
- 🌈 **彩色球体**：`/navigation_test/error_markers` (历史误差热力图)
  - 蓝色球体 = 小误差
  - 红色球体 = 大误差
  - 球体大小 = 误差大小

#### **AUV状态**：
- 🟠 **橙色箭头**：`/eca_a9/pose_gt` (AUV真实姿态)

### 📜 **终端输出报告**

每10秒系统会自动输出详细报告：

```
============================================================
📊 ESKF导航精度评估报告 (样本数: 1234)
============================================================
📍 当前位置对比:
   真值 (对齐):  [ 12.345,  -5.678,  -2.100] m
   ESKF估计:    [ 12.340,  -5.680,  -2.098] m
📏 误差统计:
   当前误差:     0.006 m
   平均误差:     0.045 m
   最大误差:     0.123 m
   最小误差:     0.001 m
   标准差:       0.023 m
============================================================
```

### 📡 **ROS话题数据**

系统发布以下话题供其他节点使用：

```bash
# 误差统计数据
/navigation_test/mean_error         # Float64: 平均误差
/navigation_test/max_error          # Float64: 最大误差  
/navigation_test/current_error_value # Float64: 当前误差

# 轨迹数据
/navigation_test/ground_truth_path   # Path: 真值轨迹
/navigation_test/estimated_path      # Path: 估计轨迹

# 可视化标记
/navigation_test/current_error       # Marker: 当前误差箭头
/navigation_test/error_markers       # MarkerArray: 误差历史
```

## 🎮 测试操作步骤

### 步骤1：**基本功能验证**

1. 启动系统后，等待约**10-15秒**让ESKF初始化完成
2. 观察终端是否显示：`🚀 改进ESKF导航系统启动成功!`
3. 检查RViz中是否出现双轨迹线条

### 步骤2：**AUV运动控制**

让AUV执行一些运动来测试导航精度：

```bash
# 在终端3中发送运动指令
# 前进运动
rostopic pub -1 /eca_a9/thrusters/0/input uuv_gazebo_ros_plugins_msgs/FloatStamped "data: 50.0"

# 转向运动  
rostopic pub -1 /eca_a9/thrusters/1/input uuv_gazebo_ros_plugins_msgs/FloatStamped "data: 20.0"

# 垂直运动
rostopic pub -1 /eca_a9/fins/0/input uuv_gazebo_ros_plugins_msgs/FloatStamped "data: 0.2"
```

或者使用键盘控制（如果可用）：
```bash
rosrun uuv_teleop uuv_keyboard_teleop.py --robot_name=eca_a9
```

### 步骤3：**精度分析**

观察不同运动模式下的导航精度：

- 📏 **静止悬停**：误差应该很小（< 0.05m）
- 🏃 **直线运动**：误差逐渐累积但保持可控
- 🔄 **转向机动**：测试姿态估计精度
- ⬆️⬇️ **垂直运动**：测试深度传感器融合效果

### 步骤4：**对比测试** (可选)

如果想与原`uuv_nav_fusion`系统对比：

```bash
# 终端4：启动原系统
roslaunch uuv_nav_fusion uuv_nav.launch robot_name:=eca_a9

# 观察两个系统的误差差异:
# - /odometry/filtered (原系统)  
# - /eskf/odometry (我们的系统)
```

## 📊 测试参数调优

### 🎛️ **ESKF参数调整**

编辑配置文件调整性能：
```bash
nano src/uuv_eskf_nav/config/eskf_params.yaml
```

关键参数：
```yaml
# IMU噪声参数
imu:
  gyro_noise_std: 0.01      # 陀螺仪噪声 [rad/s]
  accel_noise_std: 0.1      # 加速度计噪声 [m/s²]
  gyro_bias_std: 1e-5       # 陀螺仪偏差 [rad/s]
  accel_bias_std: 1e-4      # 加速度计偏差 [m/s²]

# 传感器融合参数  
dvl:
  noise_std: 0.02           # DVL速度噪声 [m/s]
depth:
  noise_std: 0.01           # 深度传感器噪声 [m]
```

### 🎯 **评估器参数调整**

编辑launch文件调整评估器：
```xml
<!-- 评估频率 -->
<param name="evaluation_frequency" value="10.0"/>

<!-- 轨迹历史长度 -->
<param name="trajectory_history_length" value="1000"/>

<!-- 启用/禁用功能 -->
<param name="enable_rviz_visualization" value="true"/>
<param name="publish_error_statistics" value="true"/>
```

## 🐛 故障排除

### 常见问题及解决方案：

#### ❌ **"等待传感器数据..."**
- **原因**：AUV仿真未完全启动或传感器插件未加载
- **解决**：重启Gazebo仿真，确保所有传感器话题正常发布

#### ❌ **RViz中看不到轨迹**
- **原因**：话题名称不匹配或帧坐标系问题
- **解决**：检查RViz配置中的话题名称，确保`Fixed Frame`设为`odom`

#### ❌ **误差异常大**
- **原因**：初始对齐失败或传感器数据质量差
- **解决**：重启测试系统，或调整噪声参数

#### ❌ **ESKF初始化失败**
- **原因**：传感器数据不足或参数配置错误
- **解决**：检查传感器话题发布状态，验证参数文件语法

## 📈 预期测试结果

### 🎯 **良好性能指标**：

- **平均误差** < 0.1m (在100m轨迹上)
- **最大误差** < 0.5m (短期峰值)
- **姿态误差** < 5° (Roll/Pitch/Yaw)
- **收敛时间** < 30s (从启动到稳定)

### 📊 **对比分析**：

相比原`uuv_nav_fusion`系统，我们的改进ESKF应该在以下方面表现更好：

1. **机动性能**：圆锥补偿改善快速转向时的精度
2. **长时间稳定性**：改进机械编排减少累积误差
3. **传感器融合**：优化的误差状态处理提升多传感器融合效果

## 🎉 测试完成

完整的导航精度测试应该包括：

- ✅ 静态精度验证（悬停测试）
- ✅ 动态跟踪精度（各种机动）
- ✅ 长时间稳定性（≥10分钟）
- ✅ 传感器故障恢复（可选）
- ✅ 与基准系统对比

测试数据可以导出进行进一步分析，或者生成技术报告用于论文发表！

## 💡 下一步优化

基于测试结果，可以考虑：

1. **算法优化**：调整Kalman滤波器参数
2. **传感器校准**：改善IMU/DVL标定
3. **环境适应**：针对特定任务优化
4. **实时性能**：优化计算效率
5. **鲁棒性**：增强故障检测与恢复

🎯 准备好开始测试了吗？
