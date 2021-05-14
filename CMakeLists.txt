cmake_minimum_required(VERSION 3.5)
project(naosoccer_visualization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nao_interfaces REQUIRED)

# Build ball_to_marker
add_executable(ball_to_marker src/ball_to_marker.cpp)
ament_target_dependencies(ball_to_marker
  rclcpp
  geometry_msgs
  visualization_msgs)

# Build joints_to_joint_state publisher
add_executable(joints_to_joint_state src/joints_to_joint_state.cpp)
ament_target_dependencies(joints_to_joint_state
  rclcpp
  sensor_msgs
  nao_interfaces)

# Install
install(TARGETS
  ball_to_marker
  joints_to_joint_state
  DESTINATION lib/${PROJECT_NAME})

foreach(dir launch rviz)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}/)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
