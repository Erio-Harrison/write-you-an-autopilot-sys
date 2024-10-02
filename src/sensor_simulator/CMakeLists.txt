cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# add lidar_simulator_node executable
add_executable(lidar_simulator_node src/lidar_simulator_node.cpp)
ament_target_dependencies(lidar_simulator_node rclcpp sensor_msgs)

ament_target_dependencies(lidar_simulator_node
  rclcpp
  sensor_msgs
)

install(TARGETS
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
