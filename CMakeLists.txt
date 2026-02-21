cmake_minimum_required(VERSION 3.8)
project(scorpio_zadanie_rekrutacyjne_software)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Cube detector component
add_library(cubeDetector SHARED src/cube_detector_node.cpp)
ament_target_dependencies(cubeDetector
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
)
rclcpp_components_register_node(cubeDetector PLUGIN "cube_detector::CubeDetectorNode" EXECUTABLE cube_detector)

# Install libraries
install(TARGETS
  cube_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
