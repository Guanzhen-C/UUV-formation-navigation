cmake_minimum_required(VERSION 2.8.3)
project(eca_a9_control)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_install_python(PROGRAMS
  scripts/acoustic_router.py
  scripts/acoustic_scheduler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
