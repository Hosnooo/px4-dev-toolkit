cmake_minimum_required(VERSION 3.8)
project(offboard_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(CTest)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_control_common REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(offboard_takeoff_handoff
  src/offboard_takeoff_handoff.cpp
)

add_executable(offboard_position
  src/offboard_position.cpp
)

add_library(se3_controller
  src/se3_controller.cpp
)

add_library(trajectory
  src/trajectory.cpp
)

add_library(px4_wrench
  src/px4_wrench.cpp
)

add_library(px4_rate
  src/px4_rate.cpp
)

add_library(lee_px4_adapter
  src/lee_px4_adapter.cpp
)

add_executable(se3
  src/se3.cpp
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

target_include_directories(se3_controller PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(trajectory PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(px4_wrench PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(px4_rate PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(lee_px4_adapter PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(trajectory PUBLIC
  yaml-cpp
)

target_include_directories(se3 PRIVATE
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

ament_target_dependencies(se3
  rclcpp
  px4_msgs
)

target_link_libraries(se3
  se3_controller
  trajectory
  px4_rate
  px4_wrench
  yaml-cpp
)

if(BUILD_TESTING)
  add_executable(test_se3_controller
    test/test_se3_controller.cpp
  )

  target_link_libraries(test_se3_controller
    se3_controller
  )

  add_test(
    NAME se3_controller_math
    COMMAND test_se3_controller
  )

  add_executable(test_trajectory
    test/test_trajectory.cpp
  )

  target_include_directories(test_trajectory PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(test_trajectory
    trajectory
  )

  target_compile_definitions(test_trajectory PRIVATE
    TRAJECTORY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/trajectories.yaml"
  )

  add_test(
    NAME trajectory_math
    COMMAND test_trajectory
  )

  add_executable(test_runtime_timing
    test/test_runtime_timing.cpp
  )

  target_include_directories(test_runtime_timing PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  add_test(
    NAME runtime_timing
    COMMAND test_runtime_timing
  )

  add_executable(test_px4_wrench
    test/test_px4_wrench.cpp
  )

  target_include_directories(test_px4_wrench PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(test_px4_wrench
    px4_wrench
  )

  add_test(
    NAME px4_wrench
    COMMAND test_px4_wrench
  )

  add_executable(test_px4_rate
    test/test_px4_rate.cpp
  )

  target_link_libraries(test_px4_rate
    px4_rate
  )

  add_test(
    NAME px4_rate
    COMMAND test_px4_rate
  )

  add_executable(test_lee_px4_adapter
    test/test_lee_px4_adapter.cpp
  )

  target_link_libraries(test_lee_px4_adapter
    lee_px4_adapter
  )

  add_test(
    NAME lee_px4_adapter
    COMMAND test_lee_px4_adapter
  )

  add_executable(test_offboard_lifecycle
    test/test_offboard_lifecycle.cpp
  )

  target_include_directories(test_offboard_lifecycle PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  add_test(
    NAME offboard_lifecycle
    COMMAND test_offboard_lifecycle
  )
endif()

install(
  TARGETS
    offboard_takeoff_handoff
    offboard_position
    se3
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    se3_controller
    trajectory
    px4_wrench
    px4_rate
    lee_px4_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Launch and recording configuration are runtime package resources. Installing
# them keeps `ros2 launch` independent of the source-tree working directory.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY "${PX4_ENV_ROOT}/config/vehicles/"
  DESTINATION share/${PROJECT_NAME}/config/vehicles
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
