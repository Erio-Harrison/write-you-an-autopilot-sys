cmake_minimum_required(VERSION 3.8)
project(network_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 手动查找 ZeroMQ
find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)

if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "ZeroMQ not found. Please install libzmq3-dev")
endif()

include_directories(
  include
  ${ZMQ_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/zeromq_adapter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${ZMQ_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rclcpp)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()