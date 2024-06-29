cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# add gps_simulator executable
add_executable(gps_simulator src/gps_simulator_node.cpp)
ament_target_dependencies(gps_simulator rclcpp sensor_msgs)

# add imu_simulator executable
add_executable(imu_simulator src/imu_simulator_node.cpp)
ament_target_dependencies(imu_simulator rclcpp sensor_msgs)

install(TARGETS
  gps_simulator
  imu_simulator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
