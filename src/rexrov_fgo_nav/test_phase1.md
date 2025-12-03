# Phase 1 Verification

## 1. Build Status
The package `rexrov_fgo_nav` has been created and built successfully. 
*   **Dependencies**: PCL, ROS standard msgs, UUV msgs are linked. 
*   **Note**: `GTSAM` was not found on the system, so it has been temporarily disabled in `CMakeLists.txt` and `package.xml`. Please install GTSAM to proceed to Phase 3 (Backend).

## 2. How to Run
To verify that the node starts and subscribes to topics correctly:

1.  **Source the workspace**:
    ```bash
    source devel/setup.bash
    ```

2.  **Launch the node**:
    ```bash
    roslaunch rexrov_fgo_nav start_fgo_nav.launch robot_name:=rexrov
    ```

3.  **Verify Data Reception**:
    If a simulation is running (e.g., `roslaunch uuv_gazebo_worlds ocean_waves.launch` and `roslaunch uuv_descriptions upload_rexrov.launch`), the node will print:
    ```
    [INFO] ... IMU Received: t=...
    [INFO] ... DVL Received: t=...
    [INFO] ... Depth Received: t=...
    ```
