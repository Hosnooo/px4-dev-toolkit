cmake_minimum_required(VERSION 3.8)
project(px4_auto_takeoff_hold)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(auto_takeoff_hold
  src/auto_takeoff_hold.cpp
)

get_filename_component(
  PX4_ENV_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)

target_include_directories(auto_takeoff_hold PRIVATE
  "${PX4_ENV_ROOT}/config"
)

ament_target_dependencies(auto_takeoff_hold
  rclcpp
  px4_msgs
)

install(
  TARGETS auto_takeoff_hold
  DESTINATION lib/${PROJECT_NAME}
)

# Launch and recording configuration are runtime package resources. Installing
# them keeps `ros2 launch` independent of the source-tree working directory.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
