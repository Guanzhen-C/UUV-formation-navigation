# ECA A9 Earth Rotation Coriolis Force Implementation

## 概述

为了支持170小时长时间水下导航任务，我们在ECA A9的Fossen水动力学模型中集成了地球自转科氏力。这一修改对于长时间导航任务至关重要，因为地球自转效应在长时间内会累积产生显著的导航误差。

## 实现特性

### 1. 地球自转科氏力模型
- **WGS84地球模型**：使用标准的WGS84椭球参数
- **地球自转角速度**：7.2921151467E-5 rad/s
- **纬度相关计算**：根据任务区域纬度调整科氏力分量
- **坐标系转换**：正确处理体坐标系与导航坐标系之间的转换

### 2. 集成到Fossen模型
地球自转科氏力作为额外的力和力矩项添加到Fossen方程中：
```
τ_total = τ_damping + τ_added_mass + τ_coriolis + τ_earth_coriolis
```

### 3. 科氏力计算公式
线性科氏力：`F_coriolis = -2 * m * (ω_ie × v)`
角动量科氏力：简化模型处理角运动的地球自转效应

其中：
- `ω_ie`：地球自转角速度在导航坐标系中的投影
- `v`：AUV在导航坐标系中的速度
- `m`：AUV的质量（包括附加质量）

## 配置使用

**重要说明：从当前版本开始，所有ECA A9启动文件都默认启用地球自转科氏力功能。**

### 1. 自动启用科氏力
ECA A9的所有启动文件现在自动包含科氏力功能，默认配置：
- **默认纬度**：18.25°N（0.3183弧度，三亚）
- **自动启用**：所有demo、控制、测试启动文件都包含科氏力

### 2. 自定义任务纬度
可以通过所有启动文件的 `mission_latitude` 参数指定纬度：

#### 标准启动文件
```bash
# 单AUV演示（指定纬度为56°N = 0.9774弧度）
roslaunch eca_a9_gazebo start_demo_auv_control.launch mission_latitude:=0.9774

# 遥控演示
roslaunch eca_a9_gazebo start_demo_teleop.launch mission_latitude:=0.9774

# 组合导航演示
roslaunch eca_a9_control start_demo_auv_control_with_fusion.launch mission_latitude:=0.9774
```

#### 直接使用upload启动文件
```bash
roslaunch eca_a9_description upload_eca_a9.launch mission_latitude:=0.9774
```

### 3. 常用纬度值参考
```bash
# 北海（56°N）
mission_latitude:=0.9774

# 地中海（36°N）  
mission_latitude:=0.6283

# 太平洋（35°N）
mission_latitude:=0.6109

# 默认值（18.25°N 三亚）
mission_latitude:=0.3183
```

### 4. 特殊启动文件
对于专门的科氏力测试，可以使用：
```bash
roslaunch eca_a9_gazebo eca_a9_empty_world_earth_coriolis.launch mission_latitude_deg:=60.0
```

### 5. URDF配置（内部）
在URDF配置中科氏力已默认启用：
```xml
<hydrodynamic_model>
  <type>fossen</type>
  <enable_earth_coriolis>true</enable_earth_coriolis>
  <vehicle_latitude>${mission_latitude}</vehicle_latitude>
  <!-- 其他Fossen参数... -->
</hydrodynamic_model>
```

## 典型任务区域配置

### 北海任务（推荐用于170小时任务）
```xml
<arg name="mission_latitude_deg" default="56.0"/>  <!-- 56° N -->
```

### 地中海任务
```xml
<arg name="mission_latitude_deg" default="36.0"/>  <!-- 36° N -->
```

### 太平洋任务
```xml
<arg name="mission_latitude_deg" default="35.0"/>  <!-- 35° N -->
```

## 调试和监控

### 1. 监控地球科氏力
```bash
rostopic echo /eca_a9/hydrodynamics/earth_coriolis_force
```

### 2. 力分量分析
系统提供详细的力分解：
- `damping_force/torque`：阻尼力/力矩
- `added_mass_force/torque`：附加质量力/力矩
- `coriolis_force/torque`：流体科氏力/力矩
- `earth_coriolis_force/torque`：地球自转科氏力/力矩（新增）

## 物理意义和影响

### 1. 舒勒振荡
地球自转科氏力的主要影响是引入舒勒振荡，其周期约为84.4分钟。对于170小时的任务，会经历约120个舒勒周期，累积误差不可忽略。

### 2. 纬度依赖性
- **赤道（0°）**：水平科氏力为零，垂直分量最大
- **极地（90°）**：水平科氏力最大，垂直分量为零
- **中纬度（30-60°）**：水平和垂直分量都很显著

### 3. 速度依赖性
科氏力与AUV速度成正比，对于典型的AUV航行速度（1-3 m/s），科氏加速度约为10^-4 m/s²量级。

## 注意事项

### 1. 计算精度
- 地球自转科氏力是小量效应，需要高精度数值计算
- 建议使用双精度浮点数进行所有相关计算

### 2. 坐标系一致性
- 确保导航坐标系定义与地球坐标系正确对应
- 注意东北天（ENU）与北东地（NED）坐标系的区别

### 3. 适用范围
- 主要适用于长时间（>10小时）的导航任务
- 对于短时间任务，可以关闭此功能以节省计算资源

## 验证方法

### 1. 数值验证
与理论科氏力公式对比：
```
F_theory = -2 * m * ω_earth * v * sin(latitude)
```

### 2. 仿真验证
在不同纬度下运行长时间仿真，观察轨迹偏移是否符合理论预期。

### 3. 比较测试
对比启用/禁用地球科氏力的导航精度差异。

## 技术支持

如需调整模型参数或适配其他纬度，请参考以下文件：
- `HydrodynamicModel.hh`：头文件声明
- `HydrodynamicModel.cc`：实现文件
- `eca_a9_base.xacro`：URDF配置
