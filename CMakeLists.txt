cmake_minimum_required(VERSION 3.0.2)
project(virtual_dc_motor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/virtual_dc_motor
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(virtual_dc_motor src/virtual_dc_motor.cpp)

target_link_libraries(virtual_dc_motor ${catkin_LIBRARIES})
