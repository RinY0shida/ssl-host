cmake_minimum_required(VERSION 3.8)
project(ssl_robot_controller_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*.action)
file(GLOB_RECURSE msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
