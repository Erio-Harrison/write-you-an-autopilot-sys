cmake_minimum_required(VERSION 3.8)
project(auto_drive_network_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(network_comm REQUIRED)
find_package(auto_drive_msgs REQUIRED)

# 手动查找 ZeroMQ (如果需要的话)
find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)

if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "ZeroMQ not found. Please install libzmq3-dev")
endif()

# 添加头文件目录
include_directories(
  include
  ${ZMQ_INCLUDE_DIR}
)

# 添加可执行文件
add_executable(network_bridge_node
  src/network_bridge_node.cpp
)

# 链接依赖库
ament_target_dependencies(network_bridge_node
  rclcpp
  network_comm
  auto_drive_msgs
)

target_link_libraries(network_bridge_node
  ${ZMQ_LIBRARY}
)

# 安装可执行文件
install(TARGETS
  network_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()