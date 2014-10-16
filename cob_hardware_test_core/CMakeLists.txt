cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_test_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_python_setup()


# Generate services in the 'srv' folder
add_service_files(
  FILES
  Dialog.srv
  TestTrigger.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages()


install(PROGRAMS test/component_test_all.py test/component_test.py test/daily_morning_show.py test/helper.py test/joy_test.py test/sensor_test_all_threaded.py test/light_test.py test/sound_test.py test/tactile_test.py test/teleop_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)
