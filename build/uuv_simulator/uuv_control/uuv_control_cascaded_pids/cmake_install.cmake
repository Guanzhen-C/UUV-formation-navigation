# Install script for directory: /home/qsk/catkin_ws/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/qsk/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uuv_control_cascaded_pid" TYPE FILE FILES "/home/qsk/catkin_ws/devel/include/uuv_control_cascaded_pid/AccelerationControlConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uuv_control_cascaded_pid" TYPE FILE FILES "/home/qsk/catkin_ws/devel/include/uuv_control_cascaded_pid/PositionControlConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uuv_control_cascaded_pid" TYPE FILE FILES "/home/qsk/catkin_ws/devel/include/uuv_control_cascaded_pid/VelocityControlConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/uuv_control_cascaded_pid" TYPE FILE FILES "/home/qsk/catkin_ws/devel/lib/python3/dist-packages/uuv_control_cascaded_pid/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/qsk/catkin_ws/devel/lib/python3/dist-packages/uuv_control_cascaded_pid/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/uuv_control_cascaded_pid" TYPE DIRECTORY FILES "/home/qsk/catkin_ws/devel/lib/python3/dist-packages/uuv_control_cascaded_pid/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/uuv_control_cascaded_pid.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_cascaded_pid/cmake" TYPE FILE FILES
    "/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/uuv_control_cascaded_pidConfig.cmake"
    "/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/uuv_control_cascaded_pidConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_cascaded_pid" TYPE FILE FILES "/home/qsk/catkin_ws/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_control_cascaded_pid" TYPE PROGRAM FILES "/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/AccelerationControl.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_control_cascaded_pid" TYPE PROGRAM FILES "/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/PositionControl.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_control_cascaded_pid" TYPE PROGRAM FILES "/home/qsk/catkin_ws/build/uuv_simulator/uuv_control/uuv_control_cascaded_pids/catkin_generated/installspace/VelocityControl.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_cascaded_pid" TYPE DIRECTORY FILES
    "/home/qsk/catkin_ws/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/launch"
    "/home/qsk/catkin_ws/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/config"
    "/home/qsk/catkin_ws/src/uuv_simulator/uuv_control/uuv_control_cascaded_pids/cfg"
    REGEX "/[^/]*\\~$" EXCLUDE)
endif()

