cmake_minimum_required(VERSION 3.0.2)
project(uuv_eskf_nav_3)

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

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  uuv_sensor_ros_plugins_msgs_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Build Navigation Node
add_executable(eskf_navigation_node_3 src/eskf_navigation_node.cpp)
target_link_libraries(eskf_navigation_node_3 ${PROJECT_NAME} ${catkin_LIBRARIES})

# 测试文件已删除，如需测试功能请重新添加相应的测试文件

## Install Python scripts
catkin_install_python(PROGRAMS
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
