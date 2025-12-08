# 水下AUV编队仿真与导航系统

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

基于 ROS Noetic 和 Gazebo 的水下自主航行器（AUV）编队仿真与高精度导航系统。本项目实现了 9 艘 AUV 编队的误差状态卡尔曼滤波（ESKF）导航算法，支持长时间海上任务仿真。

## 目录

- [项目特性](#项目特性)
- [系统架构](#系统架构)
- [环境要求](#环境要求)
- [安装指南](#安装指南)
- [快速开始](#快速开始)
- [包结构说明](#包结构说明)
- [导航系统详解](#导航系统详解)
- [仿真场景](#仿真场景)
- [载具模型](#载具模型)
- [数据分析工具](#数据分析工具)
- [配置参数](#配置参数)
- [开发指南](#开发指南)
- [常见问题](#常见问题)
- [参考资料](#参考资料)

## 项目特性

### 核心功能

- **高精度 ESKF 导航**：15 维状态估计（位置、速度、姿态、IMU 偏差）
- **地形匹配辅助导航 (TAN)**：基于多波束/侧扫声纳的粒子滤波定位，实现**亚米级 (<1m)** 长航时无漂移导航。
- **多 AUV 编队仿真**：支持 9 艘 AUV 3×3 田字阵型编队
- **完整物理仿真**：基于 Fossen 方程的 6 自由度水下动力学
- **多传感器融合**：IMU、DVL、深度计、声学测距集成
- **长时间仿真支持**：支持 170 小时以上连续仿真

### 技术亮点

- **GPU 加速粒子滤波**：基于 PyTorch 的并行计算，支持 **10万+ 粒子** 实时解算 (RTX 3070 Ti)。
- **真实地形集成**：支持 GEBCO 真实海底高程数据 (.asc) 直接导入 Gazebo。
- **闭环融合架构**：地形匹配结果反馈至 ESKF，消除惯导累积误差。
- 地球自转科里奥利力补偿
- 圆锥误差（Coning）和划桨效应（Sculling）补偿
- 自适应协方差管理

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo 仿真环境                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   AUV 1-3   │  │   AUV 4-6   │  │   AUV 7-9   │   3×3 编队   │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
└─────────┼────────────────┼────────────────┼─────────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      传感器仿真层                                │
│   IMU (导航级)  │  DVL (多普勒)  │  深度计  │  声学测距           │
│   MBES/SSS (声纳点云 & 测距)  ----->  地形匹配模块 (GPU)          │
└─────────────────────────────────────────────────────────────────┘
          │                │                │           │
          ▼                ▼                ▼           │ (位置修正)
┌───────────────────────────────────────────────────────▼─────────┐
│                    ESKF 导航系统 (×9)                            │
│   状态预测  │  测量更新  │  误差注入  │  偏差估计                  │
└─────────────────────────────────────────────────────────────────┘
```

## 环境要求

环境配置请参考 [UUV Simulator 官方文档](https://uuvsimulator.github.io/)。
**地形匹配模块额外依赖**:
*   Python 3.8+
*   PyTorch (带 CUDA 支持)
*   OpenCV (Python)

## 安装指南

### 1. 克隆仓库

```bash
cd ~/catkin_ws/src
git clone <repository-url> .
```

### 2. 安装依赖

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install torch torchvision torchaudio # 需根据CUDA版本选择
pip3 install opencv-python
```

### 3. 编译工作空间

```bash
# 使用 catkin_tools
catkin build

# 或使用 catkin_make
catkin_make -j4
```

### 4. 配置环境

```bash
# 添加到 ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 快速开始

### 地形匹配辅助导航演示 (Sub-meter Accuracy Demo)

本演示展示了如何利用侧扫声纳和真实海图实现无漂移导航。

**步骤 1：启动仿真环境与控制**
加载真实 GEBCO 地形，生成 AUV 并开始圆周运动。
```bash
roslaunch uuv_eskf_nav start_demo_circle.launch
```

**步骤 2：启动 ESKF 导航系统**
```bash
roslaunch uuv_eskf_nav eskf_navigation.launch robot_name:=eca_a9
```

**步骤 3：启动 GPU 加速地形匹配节点**
开启 10万粒子并行滤波。
```bash
roslaunch uuv_eskf_nav start_terrain_matching_gpu.launch
```

**步骤 4：查看精度对比**
绘制 ESKF 误差与地形匹配误差曲线。
```bash
rqt_plot /eca_a9/terrain_nav/error_norm/data /eca_a9/eskf/error_norm/data
```

### 启动单个 AUV 仿真

```bash
# 终端 1：启动 Gazebo 仿真环境
roslaunch uuv_gazebo_worlds ocean_waves.launch

# 终端 2：生成 RexROV 载具
roslaunch uuv_descriptions upload_rexrov.launch

# 终端 3：启动 ESKF 导航
roslaunch uuv_eskf_nav_1 eskf_navigation.launch
```

### 启动 9 艘 AUV 编队仿真

完整的 9 艘 AUV 编队仿真需要按以下顺序启动（每个命令在单独终端中运行）：

**步骤 1：启动 Gazebo 仿真环境**

```bash
roslaunch uuv_gazebo_worlds ocean_waves.launch
```

**步骤 2：生成 9 艘 AUV 编队**

```bash
roslaunch eca_a9_gazebo start_demo_eight_layer_auv_control.launch
```

**步骤 3：启动声学通信网络**

```bash
roslaunch eca_a9_control start_acoustic_network.launch
```

**步骤 4：启动各 AUV 的 ESKF 导航节点**

```bash
roslaunch uuv_eskf_nav_1 eskf_navigation.launch
roslaunch uuv_eskf_nav_2 eskf_navigation.launch
roslaunch uuv_eskf_nav_3 eskf_navigation.launch
roslaunch uuv_eskf_nav_4 eskf_navigation.launch
roslaunch uuv_eskf_nav_5 eskf_navigation.launch
roslaunch uuv_eskf_nav_6 eskf_navigation.launch
roslaunch uuv_eskf_nav_7 eskf_navigation.launch
roslaunch uuv_eskf_nav_8 eskf_navigation.launch
roslaunch uuv_eskf_nav_9 eskf_navigation.launch
```

**步骤 5：启动各 AUV 的增强评估器（可选）**

```bash
roslaunch uuv_eskf_nav_1 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_2 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_3 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_4 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_5 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_6 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_7 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_8 enhanced_evaluator.launch
roslaunch uuv_eskf_nav_9 enhanced_evaluator.launch
```

**各 AUV 话题命名空间**

```bash
# AUV1 的话题
/auv1/eskf/odometry/filtered
/auv1/eskf/pose
/auv1/imu/data
/auv1/dvl/twist
/auv1/pressure

# AUV2 ~ AUV9 以此类推
/auv2/eskf/odometry/filtered
...
/auv9/eskf/odometry/filtered
```

### 其他演示示例

```bash
# ECA A9 单 AUV 控制演示
roslaunch eca_a9_gazebo start_demo_auv_control.launch

# 多层 AUV 控制演示
roslaunch eca_a9_gazebo start_demo_multi_layer_auv_control.launch
```

## 包结构说明

```
src/
├── uuv_simulator-noetic/       # UUV Simulator 仿真框架
│   ├── uuv_gazebo_plugins/     # Gazebo 物理插件
│   ├── uuv_sensor_plugins/     # 传感器仿真插件
│   ├── uuv_control/            # 控制系统
│   ├── uuv_gazebo_worlds/      # 仿真场景
│   ├── uuv_descriptions/       # 模型描述文件
│   └── ...
├── eca_a9/                     # ECA A9 AUV 模型
│   ├── eca_a9_description/     # 模型描述
│   ├── eca_a9_control/         # 控制系统
│   └── eca_a9_gazebo/          # Gazebo 仿真
├── uuv_eskf_nav_1~9/           # 9 个 AUV 的 ESKF 导航系统
└── uuv_nav_fusion/             # 导航融合框架
```

## 导航系统详解

### ESKF 状态向量

15 维误差状态向量：

| 索引 | 状态 | 说明 |
|------|------|------|
| 0-2 | δp | 位置误差 (m) |
| 3-5 | δv | 速度误差 (m/s) |
| 6-8 | δθ | 姿态误差 (rad) |
| 9-11 | δbg | 陀螺仪偏差 (rad/s) |
| 12-14 | δba | 加速度计偏差 (m/s²) |

### 传感器规格

| 传感器 | 参数 | 典型值 |
|--------|------|--------|
| IMU 陀螺仪 | 零偏稳定性 | 0.001°/h |
| IMU 加速度计 | 噪声密度 | 1.4×10⁻⁴ m/s²/√Hz |
| DVL | 测速精度 | ±0.1% |
| 深度计 | 精度 | ±0.5 m |
| 声学测距 | 范围 | 12 km |

### 关键算法

1. **惯性导航递推**
   - 姿态更新（四元数）
   - 速度更新（重力补偿）
   - 位置更新

2. **误差状态预测**
   - 状态转移矩阵构建
   - 过程噪声传播

3. **测量更新**
   - DVL 速度观测
   - 深度观测
   - 声学测距观测

4. **误差补偿**
   - 圆锥误差补偿
   - 划桨效应补偿
   - 地球自转补偿

## 仿真场景

### 可用场景列表

| 场景名称 | 文件 | 说明 |
|---------|------|------|
| 空场景 | `empty_underwater.world` | 基础测试环境 |
| 海浪场景 | `ocean_waves.world` | 带波浪效果 |
| Mangalia | `mangalia.world` | 罗马尼亚真实地形 |
| 湖泊 | `lake.world` | 淡水湖环境 |
| BOP 面板 | `subsea_bop_panel.world` | 海底设备场景 |

### 启动场景

```bash
# 空场景
roslaunch uuv_gazebo_worlds empty_underwater_world.launch

# 海浪场景
roslaunch uuv_gazebo_worlds ocean_waves.launch

# 指定场景
roslaunch uuv_gazebo start_uuv_sim.launch world_name:=mangalia
```

## 载具模型

### ECA A9 (AUV)

鱼雷型 AUV，本项目主要使用的载具。

```bash
# 生成载具
roslaunch eca_a9_description upload_eca_a9.launch
```

### RexROV

UUV Simulator 内置的通用型 AUV 平台。

```bash
# 生成载具
roslaunch uuv_descriptions upload_rexrov.launch
```

## 数据分析工具

仿真数据以 ROS Bag 格式保存，`scripts/` 目录提供了数据处理和绘图脚本：

| 脚本 | 功能 |
|------|------|
| `bag_to_excel.py` | 将 bag 文件转换为 Excel |
| `plot_all_errors.py` | 绘制所有 AUV 的导航误差曲线 |
| `plot_relative_distance_errors.py` | 绘制相对距离误差曲线 |
| `analyze_eskf_logs.py` | ESKF 日志综合分析 |
| `check_excel_files.py` | 检查 Excel 文件完整性 |

使用示例：

```bash
# bag 转 Excel
python3 scripts/bag_to_excel.py <bag_file> <output_dir>

# 绘制误差曲线
python3 scripts/plot_all_errors.py <excel_dir>

# 绘制相对距离误差
python3 scripts/plot_relative_distance_errors.py <excel_dir>
```

## 配置参数

### ESKF 参数配置

配置文件位置：`src/uuv_eskf_nav_*/config/eskf_params.yaml`

```yaml
# IMU 参数
imu:
  gyro_noise: 0.00000029      # rad/s/√Hz
  accel_noise: 0.00014        # m/s²/√Hz
  gyro_bias_instability: 0.001 # °/h
  accel_bias_instability: 0.01 # mg

# DVL 参数
dvl:
  velocity_noise: 0.001       # m/s
  range: 12000                # m

# 深度计参数
depth:
  noise: 0.5                  # m

# 滤波器参数
filter:
  update_rate: 100            # Hz
  enable_coning: true
  enable_sculling: true
```

### Launch 文件参数

```bash
roslaunch uuv_eskf_nav_1 eskf_navigation.launch \
    enable_rviz:=true \
    enable_evaluator:=true \
    config_file:=/path/to/custom/config.yaml
```

## 开发指南

### 添加新的 AUV 导航节点

1. 复制现有导航包：
```bash
cp -r src/uuv_eskf_nav_1 src/uuv_eskf_nav_10
```

2. 修改 `CMakeLists.txt` 和 `package.xml` 中的包名

3. 更新节点命名空间配置

4. 编译并测试

### 代码规范

- C++ 代码遵循 Google C++ Style Guide
- Python 代码遵循 PEP 8
- 使用 `-O3` 优化编译导航算法

### 调试技巧

```bash
# 查看 TF 树
rosrun tf view_frames

# 监控话题
rostopic hz /auv1/eskf/odometry

# 记录数据
rosbag record -a -O experiment_$(date +%Y%m%d_%H%M%S).bag
```

## 常见问题

### Q: Gazebo 启动时崩溃

**A**: 检查 GPU 驱动和 Gazebo 版本兼容性：
```bash
# 使用软件渲染
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

### Q: ESKF 导航发散

**A**: 检查以下项目：
1. 传感器话题是否正常发布
2. IMU 频率是否达到 100Hz
3. 初始位姿是否合理
4. 协方差参数是否正确配置

### Q: 编译时找不到依赖

**A**: 确保安装所有依赖：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Q: 多 AUV 仿真卡顿

**A**: 调整仿真参数：
```bash
# 降低物理更新频率
roslaunch uuv_gazebo_worlds ocean_waves.launch \
    physics_update_rate:=500
```

## 参考资料

### 学术文献

1. Fossen, T.I. (2011). *Handbook of Marine Craft Hydrodynamics and Motion Control*
2. Groves, P.D. (2013). *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*
3. Sola, J. (2017). *Quaternion kinematics for the error-state Kalman filter*

### 在线资源

- [UUV Simulator 官方文档](https://uuvsimulator.github.io/)
- [ROS Noetic 文档](http://wiki.ros.org/noetic)
- [Gazebo 教程](http://gazebosim.org/tutorials)

### 相关项目

- [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) - 原始 UUV 仿真框架
- [rexrov2](https://github.com/uuvsimulator/rexrov2) - RexROV2 ROV 模型

## 版本历史

| 日期 | 说明 |
|------|------|
| 2025-12-02 | 精简版，清理冗余文件 |
| 2025-10-23 | ESKF 导航系统终版 |
| 2025-10-11 | 数据保存改为 bag 格式，增加数据处理脚本 |
| 2025-09-22 | DVL/深度计/互测距加误差噪声 |
| 2025-09-18 | 完成田字阵型，9 个 AUV 配备导航节点 |
| 2025-09-17 | 加入科里奥利力补偿，仿真加速 |
| 2025-09-11 | 加入声学调制解调器 |

## 许可证

本项目基于 Apache License 2.0 开源。详见 [LICENSE](LICENSE) 文件。

## 贡献者

- 项目维护者：Guanzhen-C

## 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。
