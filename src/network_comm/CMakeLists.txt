cmake_minimum_required(VERSION 3.8)
project(auto_drive_network_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_drive_msgs REQUIRED)
find_package(network_comm REQUIRED)

add_executable(network_bridge_node 
  src/network_bridge_node.cpp
)

target_include_directories(network_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(network_bridge_node
  rclcpp
  auto_drive_msgs
  network_comm
)

install(TARGETS network_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()