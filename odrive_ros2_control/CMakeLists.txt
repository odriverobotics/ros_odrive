cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(../odrive_base/include)

ament_auto_add_library(
  odrive_ros2_control_plugin SHARED
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp
)

ament_target_dependencies(odrive_ros2_control_plugin
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

target_compile_features(odrive_ros2_control_plugin PRIVATE cxx_std_20)

install(TARGETS odrive_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
