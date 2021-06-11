cmake_minimum_required(VERSION 3.8)
project(naosoccer_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(rviz_common REQUIRED)
find_package(class_loader REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nao_interfaces REQUIRED)

# Required when using Qt's moc system, refer to Qt with CMake
set(CMAKE_AUTOMOC ON)

# Build eye_leds_panel
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

add_library(eye_leds_panel SHARED
  src/eye_leds_panel.cpp
  include/naosoccer_rviz_plugins/eye_leds_panel.hpp)

ament_target_dependencies(eye_leds_panel
  rclcpp
  Qt5Core
  rviz_common
  class_loader
  std_msgs
  nao_interfaces)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS eye_leds_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(eye_leds_panel)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
