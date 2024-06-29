cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_drive_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 添加可执行文件
add_executable(gps_imu_fusion_node
  src/gps_imu_fusion_node.cpp
)

# 链接依赖库
ament_target_dependencies(gps_imu_fusion_node
  rclcpp
  auto_drive_msgs
  sensor_msgs
)

# 安装可执行文件
install(TARGETS
  gps_imu_fusion_node
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