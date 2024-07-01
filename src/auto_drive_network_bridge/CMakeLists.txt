cmake_minimum_required(VERSION 3.8)
project(auto_drive_network_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(network_comm REQUIRED)
find_package(auto_drive_msgs REQUIRED)

# Manually find ZeroMQ (if needed)
find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)

if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "ZeroMQ not found. Please install libzmq3-dev")
endif()

# nlohmann_json
include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp
  DOWNLOAD_NO_EXTRACT TRUE
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/json
)
FetchContent_MakeAvailable(json)

# Add include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/json
  ${ZMQ_INCLUDE_DIR}
)

# Add executables
add_executable(network_bridge_node
  src/network_bridge_node.cpp
)
add_executable(mock_server_node
  src/mock_server_node.cpp
)

# Link dependencies
ament_target_dependencies(network_bridge_node
  rclcpp
  network_comm
  auto_drive_msgs
)
ament_target_dependencies(mock_server_node
  rclcpp
  network_comm
  auto_drive_msgs
)

target_link_libraries(network_bridge_node ${ZMQ_LIBRARY})
target_link_libraries(mock_server_node ${ZMQ_LIBRARY})

target_link_libraries(network_bridge_node ${OPENSSL_LIBRARIES})
target_link_libraries(mock_server_node ${OPENSSL_LIBRARIES})


# Install executables
install(TARGETS
  network_bridge_node
  mock_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Install json.hpp
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/json/json.hpp
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
