cmake_minimum_required(VERSION 3.0.2)
project(rexrov_fgo_nav)

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
  tf2_eigen
  uuv_sensor_ros_plugins_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")
message(STATUS "GTSAM_LIBRARIES: ${GTSAM_LIBRARIES}")
message(STATUS "GTSAM_LIBRARY: ${GTSAM_LIBRARY}")

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs tf2_ros pcl_ros
  DEPENDS GTSAM PCL OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/SensorFrontend.cpp
  src/SonarProcessor.cpp
  src/SlidingWindowGraph.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${GTSAM_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(fgo_node src/fgo_node.cpp)
target_link_libraries(fgo_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${GTSAM_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_lib fgo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS
  scripts/navigation_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
