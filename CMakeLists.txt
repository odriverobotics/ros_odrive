cmake_minimum_required(VERSION 3.8)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMessage.msg"
  "msg/ControllerStatus.msg"
  "msg/ODriveStatus.msg"
  "srv/AxisState.srv"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(odrive_can_node
  src/epoll_event_loop.cpp
  src/socket_can.cpp
  src/odrive_can_node.cpp
  src/main.cpp
  include/odrive_can_node.hpp)

ament_target_dependencies(odrive_can_node
  rclcpp
  std_srvs
)

target_compile_features(odrive_can_node PRIVATE cxx_std_20)

install(
  TARGETS odrive_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odrive_can_node "${cpp_typesupport_target}")

ament_package()
