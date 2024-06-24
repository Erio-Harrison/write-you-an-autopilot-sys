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
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# 添加 lidar_simulator_node
add_executable(lidar_simulator_node src/lidar_simulator_node.cpp)
ament_target_dependencies(lidar_simulator_node
  rclcpp
  auto_drive_msgs
  sensor_msgs
)

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

target_link_libraries(obstacle_tracking_node Eigen3::Eigen)

# 安装可执行文件
install(TARGETS
  obstacle_detection_node
  obstacle_tracking_node
  lidar_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()