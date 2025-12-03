# Phase 3 Verification: Full System Integration

## 1. Build Status
The `rexrov_fgo_nav` package is fully integrated and built.
*   **Frontend**: IMU, DVL, Depth, and Sonar (MBES) data processing is active.
*   **Backend**: GTSAM Factor Graph (ISAM2) fuses all sensor inputs.
*   **Loop**: 
    1.  IMU drives Preintegration.
    2.  MBES NDT registration triggers `OdomCallback`.
    3.  `fgo_node` injects relative pose factors into the graph.
    4.  Graph updates at 10Hz, publishing optimized odometry.

## 2. How to Run

1.  **Launch Simulation**:
    ```bash
    roslaunch rexrov_fgo_nav start_sonar_demo.launch
    ```
    *   This loads `ocean_waves` world.
    *   Spawns RexROV with Down-looking MBES.
    *   Starts FGO Node.
    *   Opens RViz.

2.  **Operation**:
    *   Use the joystick (if available) or teleop node to move the ROV.
    *   ```bash
        roslaunch uuv_teleop uuv_teleop.launch model_name:=rexrov
        ```
    *   Observe `/rexrov/fgo_odom` (Green Path) vs Ground Truth in RViz.
    *   Observe Point Cloud accumulation in RViz.

3.  **Verification Checklist**:
    *   [ ] **PointCloud**: Is the seafloor visible in RViz (`/rexrov/mbes_cloud`)?
    *   [ ] **Odometry**: Does `/rexrov/fgo_odom` move smoothly with the vehicle?
    *   [ ] **Drift**: Does FGO show less drift than pure dead reckoning (visual check)?

## 3. Troubleshooting
*   **No Point Cloud**: Check if `rexrov_sonar_custom.xacro` loaded correctly. Pitch should be 90 deg (down).
*   **NDT Failed**: Ensure the ROV is close enough to the seafloor (Altitude < 50m).
*   **Graph Unstable**: Check initialization logic in `SlidingWindowGraph.cpp`.
