# RexROV Sliding Window Factor Graph Navigation (FGO-Nav) - Technical Roadmap

## 1. 项目目标 (Project Objective)
为 RexROV 单个 UUV 实现基于 **滑动窗口因子图 (Sliding Window Factor Graph)** 的高精度导航定位系统。
该系统将在现有的 IMU、DVL、深度计融合基础上，新增 **多波束测深侧扫声呐 (Multibeam Echosounder / Sidescan)** 的扫描匹配约束。
**核心要求**：
*   使用 `gazebo_ros_image_sonar.cpp` 插件模拟声呐。
*   配置为 **下视 (Down-looking)** 模式，模拟多波束测深。
*   同时利用 **图像数据 (强度/Backscatter)** 和 **点云数据 (高程/Geometry)** 进行配准。

## 2. 技术架构 (Technical Architecture)

### 2.1 传感器配置 (Sensor Setup)
*   **平台**: RexROV (Gazebo Simulation)
*   **核心传感器**: 
    *   **IMU**: 50Hz, 提供角速度与加速度。
    *   **DVL**: 5Hz, 提供机体速度。
    *   **深度计**: 1Hz, 提供 Z 轴约束。
    *   **多波束测深声呐 (MBES)**: 
        *   **实现**: 复用 `gazebo_ros_image_sonar.cpp`，通过 URDF 旋转 90 度朝下安装。
        *   **话题**: `mbes` (命名空间下)。
        *   **数据流**: 
            1.  `mbes/image_raw` (Float32): 深度图像 (Range/Depth Image)。
            2.  `mbes/image_raw_multibeam` (Mono8): 声呐强度图像 (Intensity Image)。

### 2.2 传感器前端 (Sensor Frontend)

*   **惯性/航位推算**: IMU 预积分 + DVL 速度观测 + 深度约束。
*   **MBES 扫描匹配 (MBES Scan Matching)**:
    1.  **数据同步 (Sync)**: 
        *   同步订阅 `mbes/image_raw` (几何) 和 `mbes/image_raw_multibeam` (纹理)。
    2.  **XYZI 点云重构 (Reconstruction)**: 
        *   **几何恢复**: 根据相机内参 (FOV) 将深度图反投影到 3D 空间 (Sensor Frame)。
        *   **强度融合**: 将强度图的像素值赋给对应点的 Intensity 通道。
        *   **坐标转换**: Sensor Frame (X-Down) -> Body Frame (X-Forward) -> Stabilized Frame (Roll/Pitch compensated)。
    3.  **特征增强与滤波**: 
        *   **强度阈值**: 仅保留高回波强度的海床特征点 (岩石、管道等)，滤除水体噪声。
        *   **体素滤波**: 下采样以提高配准效率。
    4.  **配准 (Registration)**: 
        *   使用 **GICP (Generalized-ICP)** 或 **NDT**。
        *   输入: 带有强度权重的点云 (或仅利用几何特征，视强度对几何的影响而定)。
        *   目标: 计算当前 Swath 与 局部地图 (Local Map) 的相对位姿。
    5.  **输出**: 相对位姿因子 (BetweenFactor)。

### 2.3 因子图后端 (Factor Graph Backend)

采用 **GTSAM** 库：
*   **状态**: $X_i$ (Pose), $V_i$ (Vel), $B_i$ (Bias)。
*   **因子**:
    *   **ImuFactor**: 约束相邻关键帧。
    *   **PriorFactor (Partial)**: DVL 速度约束 $V_i$, 深度约束 $Z_i$。
    *   **BetweenFactor**: 声呐配准得到的 $\Delta T_{i,j}$。
*   **优化策略**: ISAM2 增量平滑。

## 3. 开发路线图 (Development Roadmap)

### 阶段一：环境搭建与基础移植 (Phase 1: Setup & Migration) (已完成)
- [x] **创建 Package**: `rexrov_fgo_nav`。
- [x] **基础前端**: 移植 IMU, DVL, Depth 处理。
- [x] **GTSAM 集成**: 解决依赖问题，实现基础图优化回路。

### 阶段二：声呐系统配置与前端 (Phase 2: Sonar Setup & Frontend) (进行中)
- [x] **传感器配置**: 
    - [x] 创建 `rexrov_sonar_custom.xacro`，将声呐朝下安装 (Pitch=90°)。
- [ ] **SonarFusion 节点**: 
    - [ ] 更新订阅话题为 `mbes/...`。
    - [ ] 实现深度图与强度图的融合逻辑。
    - [ ] 验证下视视角下的点云生成是否正确 (RViz 可视化)。

### 阶段三：完整系统集成 (Phase 3: Full Integration)
- [ ] **因子图接入**: 将 MBES 配准结果接入 GTSAM。
- [ ] **闭环测试**: 
    - [ ] 仿真海底地形扫描。
    - [ ] 评估定位精度。

## 4. 关键文件结构

```
src/rexrov_fgo_nav/
├── urdf/
│   └── rexrov_sonar_custom.xacro  # 包含下视 MBES 的描述文件
├── include/rexrov_fgo_nav/
│   ├── SonarProcessor.h           # MBES 数据处理 (Depth+Intensity -> Cloud -> Registration)
│   └── SlidingWindowGraph.h       # GTSAM 后端
├── src/
│   ├── SonarProcessor.cpp
│   └── SlidingWindowGraph.cpp
└── launch/
    └── start_fgo_nav.launch
```
