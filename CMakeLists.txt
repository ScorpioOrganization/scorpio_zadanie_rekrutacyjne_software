cmake_minimum_required(VERSION 3.8)
project(scorpio_zadanie_rekrutacyjne_software)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED) # Example of importing cv_bridge

include_directories(include)

# Echo node component
add_library(echoNode SHARED src/echo_node.cpp)
ament_target_dependencies(echoNode
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  # cv_bridge
)
rclcpp_components_register_node(echoNode PLUGIN "example::EchoNode" EXECUTABLE echo)

# Install libraries
install(TARGETS
  echoNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
