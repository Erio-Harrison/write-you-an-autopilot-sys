cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auto_drive_msgs REQUIRED)

# add lidar_simulator executable
add_executable(lidar_simulator src/lidar_simulator_node.cpp)
ament_target_dependencies(lidar_simulator rclcpp sensor_msgs)

# add vehicle_simulator executable
add_executable(vehicle_simulator src/vehicle_simulator_node.cpp)
ament_target_dependencies(vehicle_simulator rclcpp sensor_msgs)

ament_target_dependencies(vehicle_simulator
  rclcpp
  auto_drive_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  lidar_simulator
  vehicle_simulator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
