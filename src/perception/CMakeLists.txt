cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_drive_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
ament_target_dependencies(obstacle_detection_node
  rclcpp
  auto_drive_msgs
  sensor_msgs
)

add_executable(obstacle_tracking_node src/obstacle_tracking_node.cpp)
ament_target_dependencies(obstacle_tracking_node
  rclcpp
  auto_drive_msgs
  sensor_msgs
)

target_link_libraries(obstacle_tracking_node Eigen3::Eigen)

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