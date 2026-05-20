cmake_minimum_required(VERSION 3.5)
project(sim2d_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(sim2d_hardware_interface SHARED
  src/sim2d_hardware_interface.cpp
)

target_include_directories(sim2d_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sim2d_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  tf2
  Eigen3
)

pluginlib_export_plugin_description_file(hardware_interface sim_2d_control.xml)

ament_export_libraries(sim2d_hardware_interface)

ament_package()