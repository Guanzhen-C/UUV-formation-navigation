cmake_minimum_required(VERSION 3.0.2)
project(uuv_eskf_nav)

## Compile as C++17
add_compile_options(-std=c++17 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  uuv_sensor_ros_plugins_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs
    uuv_sensor_ros_plugins_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Build ESKF Navigation Library
add_library(${PROJECT_NAME}
  src/eskf_core.cpp
  src/sensor_manager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Build Navigation Node
add_executable(eskf_navigation_node src/eskf_navigation_node.cpp)
target_link_libraries(eskf_navigation_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Build Test Node for ESKF
add_executable(test_eskf src/test_eskf.cpp)
target_link_libraries(test_eskf ${PROJECT_NAME} ${catkin_LIBRARIES})

## Build Test Node for Measurement Update
add_executable(test_measurement_update src/test_measurement_update.cpp)
target_link_libraries(test_measurement_update ${PROJECT_NAME} ${catkin_LIBRARIES})

## Build Earth Rotation Compensation Test
add_executable(test_earth_rotation src/test_earth_rotation.cpp)
target_link_libraries(test_earth_rotation ${PROJECT_NAME} ${catkin_LIBRARIES})

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/navigation_evaluator.py
  scripts/startup_message.py
  scripts/enhanced_navigation_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
