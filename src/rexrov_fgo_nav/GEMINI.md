# RexROV FGO Navigation

## Project Overview
**`rexrov_fgo_nav`** is a ROS package implementing **Factor Graph Optimization (FGO)** for high-precision underwater navigation of the RexROV vehicle. It fuses data from multiple sensors (IMU, DVL, Depth Sensor, and Sonar) to estimate the vehicle's 6-DOF pose and velocity.

Unlike traditional EKF approaches, this system uses **GTSAM (Georgia Tech Smoothing and Mapping)** with an `ISAM2` solver to perform incremental smoothing, allowing for more robust handling of non-linearities and delayed measurements (like Sonar loop closures).

## Architecture

### Factor Graph Structure
The state vector at each keyframe $i$ consists of:
*   **Pose ($X_i$):** 3D Position + 3D Orientation ($SO(3) \times \mathbb{R}^3$).
*   **Velocity ($V_i$):** 3D Velocity in the **World Frame**.
*   **Bias ($B_i$):** IMU Accelerometer and Gyroscope biases.

### Factors & Sensors

| Sensor | Factor Type | Description |
| :--- | :--- | :--- |
| **IMU** | `gtsam::ImuFactor` | High-frequency (100Hz+) acceleration and angular velocity are **preintegrated** between keyframes to form relative constraints. |
| **DVL** | `BodyVelocityFactor` | **Custom Factor.** Constrains the velocity in the **Body Frame**. <br> Error function: $e = v_{meas} - R_{wb}^T \cdot v_{world}$. <br> This decouples velocity measurement accuracy from orientation estimation errors. |
| **Depth** | `gtsam::GPSFactor` | Constrains the Z-axis position (Depth) globally. X and Y axes are left unconstrained (high covariance). |
| **Sonar** | `gtsam::BetweenFactor` | Adds relative pose constraints ($T_{i,j}$) derived from sonar scan matching or feature tracking. Crucial for drift correction. |
| **Bias** | `gtsam::BetweenFactor` | Models bias evolution as a Random Walk process. |

### Key Classes

*   **`SlidingWindowGraph`**: The core wrapper around GTSAM's `ISAM2`. It manages the graph construction, adds factors, and performs updates. Despite the name, it maintains the full history via ISAM2 but focuses optimization on recent states.
*   **`BodyVelocityFactor`**: A custom GTSAM factor implementation that allows direct integration of body-frame DVL measurements without prior conversion to world frame.
*   **`fgo_node`**: The ROS node that handles data callbacks (`SensorFrontend`), synchronizes measurements, and triggers graph updates (default ~10Hz).

## Build & Usage

### Prerequisites
*   ROS Noetic
*   **GTSAM** (via `ros-noetic-gtsam` or source)
*   Eigen3

### Building
```bash
cd ~/catkin_ws
catkin build rexrov_fgo_nav
source devel/setup.bash
```

### Running
To launch the FGO navigation node:
```bash
roslaunch rexrov_fgo_nav start_fgo_nav.launch robot_name:=rexrov
```

To view the trajectory and DVL velocity debug info:
```bash
rostopic echo /rexrov_fgo_node/fgo_odom
rostopic echo /rexrov_fgo_node/dvl_body_velocity
```

## Conventions
*   **Coordinate Frames:** Follows ROS **REP-103** (ENU - East, North, Up).
    *   $X$: East
    *   $Y$: North
    *   $Z$: Up (Depth is negative Z or handled via transformation)
*   **Units:** SI Units (Meters, Radians, Seconds).
*   **Code Style:** ROS C++ Style Guide.
*   **GTSAM Symbols:**
    *   `X`: Pose
    *   `V`: Velocity
    *   `B`: Bias
