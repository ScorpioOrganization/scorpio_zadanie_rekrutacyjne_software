cmake_minimum_required(VERSION 3.0.2)
project(virtual_dc_motor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  roscpp
)

add_service_files(
  FILES
  getMotorJointsLengths.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/virtual_dc_motor
)

add_executable(virtual_dc_motor src/virtual_dc_motor.cpp)

add_dependencies(virtual_dc_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(virtual_dc_motor ${catkin_LIBRARIES})
