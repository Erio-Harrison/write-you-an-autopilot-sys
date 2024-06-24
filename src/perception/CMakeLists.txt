cmake_minimum_required(VERSION 3.8)
project(perception)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_drive_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加 obstacle_detection_node
add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
ament_target_dependencies(obstacle_detection_node
  rclcpp
  auto_drive_msgs
  sensor_msgs
)

# 添加 obstacle_tracking_node
add_executable(obstacle_tracking_node src/obstacle_tracking_node.cpp)
ament_target_dependencies(obstacle_tracking_node
  rclcpp
  auto_drive_msgs
  sensor_msgs
)

# 安装可执行文件
install(TARGETS
  obstacle_detection_node
  obstacle_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()