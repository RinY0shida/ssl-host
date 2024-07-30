cmake_minimum_required(VERSION 3.8)
project(ssl_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(robot_controller_node SHARED
  src/robot_controller_component.cc
  src/trajectory_controller.cc
  include/ssl_robot_controller/trajectory_controller.h
)

rclcpp_components_register_nodes(robot_controller_node
  PLUGIN "ssl_robot_controller::RobotController"
  EXECUTABLE robot_controller_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)