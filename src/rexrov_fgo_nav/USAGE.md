# Usage Guide: RexROV FGO Navigation

## 1. Overview
`rexrov_fgo_nav` implements a sliding window factor graph based navigation system for the RexROV UUV. It fuses:
*   **IMU**: High-rate Preintegration Factor.
*   **DVL**: Velocity Factor.
*   **Pressure**: Depth Factor.
*   **Sonar**: Scan Matching Odometry using NDT algorithm.

The system uses GTSAM's ISAM2 for incremental smoothing and optimization.

## 2. Prerequisites
*   **GTSAM 4.0+**: Installed via PPA or Source.
*   **PCL & OpenCV**: Standard ROS installation.
*   **UUV Simulator**: Installed.

## 3. Running the FGO Navigation Demo

Follow these steps in order. Each step requires a separate terminal.

### Step 1: Launch Simulation Environment
Start Gazebo world, spawn RexROV, and load controllers:
```bash
roslaunch rexrov_fgo_nav start_simulation.launch
```
Wait for Gazebo to fully load and stabilize.

### Step 2: Send Motion Commands
Start a helical trajectory (or other motion pattern):
```bash
roslaunch uuv_control_utils start_helical_trajectory.launch uuv_name:=rexrov n_turns:=2
```
Alternative motion patterns:
```bash
# Circular trajectory
roslaunch uuv_control_utils start_circular_trajectory.launch uuv_name:=rexrov

# Waypoint following
roslaunch uuv_control_utils send_waypoints_file.launch uuv_name:=rexrov
```

### Step 3: Launch FGO Navigation Node
Start the factor graph navigation and error evaluator:
```bash
roslaunch rexrov_fgo_nav start_fgo_nav.launch
```

Options:
```bash
# Enable/disable evaluator (default: true)
roslaunch rexrov_fgo_nav start_fgo_nav.launch enable_evaluator:=true

# Enable RViz visualization
roslaunch rexrov_fgo_nav start_fgo_nav.launch enable_rviz:=true
```

## 4. Topics

### Published by FGO Node
| Topic | Type | Description |
|-------|------|-------------|
| `/rexrov/fgo_odom` | nav_msgs/Odometry | FGO navigation estimate |

### Subscribed by FGO Node
| Topic | Type | Description |
|-------|------|-------------|
| `/rexrov/imu` | sensor_msgs/Imu | IMU data |
| `/rexrov/dvl` | uuv_sensor_ros_plugins_msgs/DVL | DVL velocity |
| `/rexrov/pressure` | sensor_msgs/FluidPressure | Pressure sensor |
| `/rexrov/mbes_sonar` | sensor_msgs/PointCloud2 | Sonar point cloud |
| `/rexrov/pose_gt` | nav_msgs/Odometry | Ground truth (for init) |

### TF Frames
The FGO node broadcasts: `world` -> `rexrov/base_link`

## 5. RViz Visualization
Use the provided RViz config:
```bash
roslaunch rexrov_fgo_nav start_fgo_nav.launch enable_rviz:=true
```

Or manually open:
```bash
rviz -d $(rospack find rexrov_fgo_nav)/config/fgo_demo.rviz
```

Key settings:
- Fixed Frame: `world`
- Add Odometry display for `/rexrov/fgo_odom`
- Add PointCloud2 for `/rexrov/sonar_cloud`

## 6. Parameters
Configuration is currently in source files:
- `SlidingWindowGraph.cpp`: IMU noise, bias noise, sensor noise models
- `SonarProcessor.cpp`: NDT parameters, scan matching settings

Future work: Move parameters to `config/fgo_params.yaml`.

## 7. Architecture

```
                    +-----------------+
                    |   SensorFrontend |
                    | (IMU/DVL/Depth)  |
                    +---------+-------+
                              |
                              v
+---------------+    +-----------------+
| SonarProcessor| -> | SlidingWindowGraph|
| (NDT Matching)|    |   (GTSAM ISAM2)  |
+---------------+    +---------+-------+
                              |
                              v
                    +-----------------+
                    |    fgo_node     |
                    | (Pub: fgo_odom) |
                    +-----------------+
```
