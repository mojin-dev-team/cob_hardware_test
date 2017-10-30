cmake_minimum_required(VERSION 2.8.3)
project(cob_test_rigs)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  launch/rqt_plot.sh
  scripts/test_fdm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch robots
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
