# 9艘AUV舰队ESKF导航系统使用说明

## 📁 包结构总览

已成功创建9个独立的ESKF导航包，每个包对应一艘AUV：

```
src/
├── uuv_eskf_nav_1/     # AUV1 导航包 (robot_name: "auv1")
├── uuv_eskf_nav_2/     # AUV2 导航包 (robot_name: "auv2")
├── uuv_eskf_nav_3/     # AUV3 导航包 (robot_name: "auv3")
├── uuv_eskf_nav_4/     # AUV4 导航包 (robot_name: "auv4")
├── uuv_eskf_nav_5/     # AUV5 导航包 (robot_name: "auv5")
├── uuv_eskf_nav_6/     # AUV6 导航包 (robot_name: "auv6")
├── uuv_eskf_nav_7/     # AUV7 导航包 (robot_name: "auv7")
├── uuv_eskf_nav_8/     # AUV8 导航包 (robot_name: "auv8")
└── uuv_eskf_nav_9/     # AUV9 导航包 (robot_name: "auv9")
```

## 🚀 启动方式

### 单个AUV启动
```bash
# 启动AUV1的导航系统
roslaunch uuv_eskf_nav_1 eskf_navigation.launch

# 启动AUV2的导航系统  
roslaunch uuv_eskf_nav_2 eskf_navigation.launch

# ... 以此类推
```

### 同时启动多个AUV（后台运行）
```bash
# 同时启动所有9个AUV的导航系统
roslaunch uuv_eskf_nav_1 eskf_navigation.launch &
roslaunch uuv_eskf_nav_2 eskf_navigation.launch &
roslaunch uuv_eskf_nav_3 eskf_navigation.launch &
roslaunch uuv_eskf_nav_4 eskf_navigation.launch &
roslaunch uuv_eskf_nav_5 eskf_navigation.launch &
roslaunch uuv_eskf_nav_6 eskf_navigation.launch &
roslaunch uuv_eskf_nav_7 eskf_navigation.launch &
roslaunch uuv_eskf_nav_8 eskf_navigation.launch &
roslaunch uuv_eskf_nav_9 eskf_navigation.launch &
```

### 启动时可选参数
```bash
# 启用RViz可视化
roslaunch uuv_eskf_nav_1 eskf_navigation.launch enable_rviz:=true

# 禁用导航评估器
roslaunch uuv_eskf_nav_1 eskf_navigation.launch enable_evaluator:=false

# 使用自定义配置文件
roslaunch uuv_eskf_nav_1 eskf_navigation.launch config_file:=/path/to/custom/config.yaml
```

## 🔧 节点命名方案（避免冲突）

每个包的节点都使用了独特的命名：

| 包名 | 主节点 | 评估器节点 | 启动信息节点 |
|------|--------|------------|--------------|
| uuv_eskf_nav_1 | eskf_navigation_1 | eskf_navigation_evaluator_1 | startup_message_1 |
| uuv_eskf_nav_2 | eskf_navigation_2 | eskf_navigation_evaluator_2 | startup_message_2 |
| ... | ... | ... | ... |
| uuv_eskf_nav_9 | eskf_navigation_9 | eskf_navigation_evaluator_9 | startup_message_9 |

## 📊 话题命名空间

每个AUV的话题会根据robot_name自动添加命名空间：

```bash
# AUV1的话题
/auv1/eskf/odometry/filtered
/auv1/eskf/pose
/auv1/imu/data
/auv1/dvl/twist
/auv1/pressure

# AUV2的话题
/auv2/eskf/odometry/filtered
/auv2/eskf/pose
/auv2/imu/data
/auv2/dvl/twist
/auv2/pressure

# ... 以此类推
```

## 🛠️ 编译验证

所有包已经成功编译，可执行文件位于：
```
devel/lib/uuv_eskf_nav_1/eskf_navigation_node_1
devel/lib/uuv_eskf_nav_2/eskf_navigation_node_2
...
devel/lib/uuv_eskf_nav_9/eskf_navigation_node_9
```

## 📋 配置文件位置

每个AUV的配置文件：
```
src/uuv_eskf_nav_1/config/eskf_params.yaml  # robot_name: "auv1"
src/uuv_eskf_nav_2/config/eskf_params.yaml  # robot_name: "auv2"
...
src/uuv_eskf_nav_9/config/eskf_params.yaml  # robot_name: "auv9"
```

## 🎯 优势特点

✅ **完全隔离** - 每个AUV有独立的包空间，无冲突风险
✅ **独立配置** - 每个AUV可以有不同的参数设置  
✅ **简单管理** - 清晰的命名规则，易于识别和维护
✅ **ROS兼容** - 完全符合ROS包管理规范
✅ **扩展性强** - 可以轻松添加更多AUV或修改单个AUV配置

## 🔍 故障排除

### 检查节点状态
```bash
# 查看所有ESKF导航节点
rosnode list | grep eskf_navigation

# 查看特定AUV的话题
rostopic list | grep auv1
```

### 查看日志
```bash
# 查看AUV1的导航节点日志
rosnode info eskf_navigation_1

# 实时查看日志
rostopic echo /auv1/eskf/odometry/filtered
```

---

**创建时间**: $(date)  
**状态**: ✅ 已创建并验证9个AUV导航包  
**编译状态**: ✅ 所有包编译成功
