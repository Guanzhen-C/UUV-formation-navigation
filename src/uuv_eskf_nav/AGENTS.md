# Repository Guidelines

## Project Structure & Module Organization
This catkin package keeps runtime code in `src/` (C++ nodes such as `eskf_core.cpp` and `sensor_manager.cpp`) with matching headers inside `include/uuv_eskf_nav/`. Python utilities (`terrain_map_server.py`, `enhanced_navigation_evaluator.py`, etc.) live under `scripts/` and should mirror the ROS node name they implement. Configuration goes in `config/` (`eskf_params.yaml` for filter tuning, `navigation_test.rviz` for visualization presets), while launch files reside in `launch/` and reference terrain assets from `models/` or the `*.asc` datasets at the repo root. ROS build artifacts stay under `build/` and `devel/`; avoid committing them.

## Build, Test, and Development Commands
```bash
catkin_make                    # Build C++ targets and install Python entry points.
source devel/setup.bash        # Overlay the workspace so ROS can locate nodes and parameters.
roslaunch uuv_eskf_nav eskf_navigation.launch robot_name:=eca_a9 enable_rviz:=true
                               # Run the ESKF stack with visualization to validate changes.
catkin_make run_tests          # Executes any gtest/rostest suites once they are added.
```

## Coding Style & Naming Conventions
Target C++17, four-space indentation, and brace-on-new-line formatting already used in `src/`. Classes use `CamelCase`, members keep the trailing underscore (`mission_latitude_`), and ROS topics, frames, and parameters stay in `lower_snake_case`. Python scripts should follow PEP 8 with explicit `#!/usr/bin/env python3` shebangs. Run `clang-format` on touched C++ files and `ruff check scripts/` before committing.

## Testing Guidelines
Add deterministic unit tests using `rostest` or gtest fixtures co-located in `test/` and hook them into `CMakeLists.txt` so `catkin_make run_tests` catches regressions. Name launch-driven tests `<feature>_test.launch` and place reusable RViz or bag configs under `config/`. Until automated coverage exists, validate new filters by replaying a known bag, comparing `/eskf/odometry/filtered` against `/robot_pose_gt`, and logging statistics with `scripts/enhanced_navigation_evaluator.py`.

## Commit & Pull Request Guidelines
Follow the existing history by using concise, imperative summaries optionally prefixed with a scope (`Feat: Implement terrain aid`, `Docs: Refresh README`). Each PR should describe motivation, testing evidence (commands + outputs), and any new parameters. Link GitHub issues when applicable, attach RViz screenshots for visualization tweaks, and request reviews from sensor fusion maintainers. Rebase on the latest `main` before submission to keep CI noise low.

## Configuration & Sensor Tips
Keep shared defaults in `config/eskf_params.yaml` conservative—override per vehicle via new YAML files named `eskf_<robot>.yaml` and reference them inside `launch/eskf_navigation.launch`. Sensitive maps (e.g., `high_res_terrain.asc`) are large; avoid duplicating them in feature branches and prefer symbolic references under `models/`. When handling custom robots, verify that incoming topics follow the `{robot_name}/imu`, `/dvl`, and `/pressure` naming pattern before extending the filter.
