execute_process(COMMAND "/home/qsk/catkin_ws/build/uuv_simulator-noetic/uuv_control/uuv_trajectory_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/qsk/catkin_ws/build/uuv_simulator-noetic/uuv_control/uuv_trajectory_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
