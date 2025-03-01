cmake_minimum_required(VERSION 3.8)
project(example_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Example Node component
add_library(exampleNode SHARED src/example_node.cpp)
ament_target_dependencies(exampleNode rclcpp std_msgs std_srvs rclcpp_components)
rclcpp_components_register_node(exampleNode PLUGIN "example_package::ExampleNode" EXECUTABLE example_node)

# Install libraries
install(TARGETS
  exampleNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
