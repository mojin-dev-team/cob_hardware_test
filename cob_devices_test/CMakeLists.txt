cmake_minimum_required(VERSION 2.8.3)
project(cob_devices_test)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  scripts/test_light.py
  scripts/udev_joy.sh
  scripts/udev_s300.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
