cmake_minimum_required(VERSION 3.8)
project(offboard_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_control_common REQUIRED)

add_executable(offboard_takeoff_handoff
  src/offboard_takeoff_handoff.cpp
)

add_executable(offboard_position
  src/offboard_position.cpp
)

get_filename_component(
  PX4_ENV_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)

target_include_directories(offboard_takeoff_handoff PRIVATE
  "${PX4_ENV_ROOT}/config"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(offboard_position PRIVATE
  "${PX4_ENV_ROOT}/config"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(offboard_takeoff_handoff
  rclcpp
  px4_msgs
  px4_control_common
)

ament_target_dependencies(offboard_position
  rclcpp
  px4_msgs
)

install(
  TARGETS
    offboard_takeoff_handoff
    offboard_position
  DESTINATION lib/${PROJECT_NAME}
)

# Launch and recording configuration are runtime package resources. Installing
# them keeps `ros2 launch` independent of the source-tree working directory.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
