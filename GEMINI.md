# Project Context: 9-Node AUV Formation Simulation (170h)

## 1. Project Overview
This is a **ROS Noetic (Catkin)** workspace dedicated to simulating a **9-node AUV (Autonomous Underwater Vehicle) formation** for a duration of **170 hours**.

The primary goal is to validate long-term formation keeping accuracy (< 500m error) using a cooperative navigation approach (ESKF + Acoustic Ranging).

### Key Documents
*   **`src/detailed_simulation_plan.md`**: The master plan for the simulation. Contains the architectural design for time acceleration (10x-50x), parallel computing strategy, TDMA acoustic communication protocol, and performance metrics. **Consult this file for all architectural decisions.**

## 2. Workspace Structure
The project follows the standard Catkin workspace layout:

*   **`src/`**: Source code.
    *   **`uuv_simulator-noetic/`**: The core Gazebo-based underwater simulation platform.
    *   **`uuv_eskf_nav/`**: Base package for Error-State Kalman Filter navigation.
    *   **`uuv_eskf_nav_[1-9]/`**: Specific navigation configurations for each of the 9 AUVs.
    *   **`uuv_nav_fusion/`**: Central fusion framework.
    *   **`eca_a9/`**: Model description for the ECA A9 AUV.
*   **`scripts/`**: Analysis tools (e.g., `analyze_eskf_logs.py`) for processing simulation logs.
*   **`logs/`**: Simulation output logs (bag files, excel exports, plots).

## 3. Simulation Architecture (Planned)
*   **Topology**: 3x3 Grid Formation.
*   **Communication**: Acoustic TDMA (Time Division Multiple Access) with a 20-second slot per vehicle.
*   **Navigation**: Cooperative ESKF (Error-State Kalman Filter).
*   **Optimization**:
    *   Time Acceleration: 10x - 50x adaptive step.
    *   Parallel Computing: Independent threads for each AUV's navigation loop.

## 4. Development & Usage

### Building the Workspace
```bash
# From the workspace root (/home/cgz/catkin_ws)
catkin build
# Or strictly:
catkin_make
```

### Sourcing the Environment
```bash
source devel/setup.bash
```

### Running Simulations (Typical Pattern)
*   **Launch Simulation**: `roslaunch <package_name> <launch_file>`
    *   *Note: Specific launch files for the 9-node sim need to be identified/created based on the plan.*
*   **Analysis**: Run scripts in `scripts/` to analyze the generated `.bag` or Excel files.

## 5. Conventions
*   **Code Style**: Follow standard ROS C++ (Google Style) and Python (PEP 8) conventions.
*   **Coordinate Frames**: Adhere to ROS REP-103 (ENU) and REP-105 (Map/Odom/Base_Link).
*   **Units**: SI units (Meters, Radians, Seconds).
