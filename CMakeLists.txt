cmake_minimum_required(VERSION 3.5)
project(ros2_sim2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(sim2d_hardware_interface SHARED
  src/sim2d_hardware_interface.cpp
  src/kinematics_solver.cpp
)

target_include_directories(sim2d_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(sim2d_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  tf2
  tf2_ros
  nav_msgs
  Eigen3
)

# --- Test executable for KinematicSolver ---
add_executable(test_kinematics_solver src/test_kinematics_solver.cpp)
target_include_directories(test_kinematics_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(test_kinematics_solver PRIVATE Eigen3::Eigen sim2d_hardware_interface)
# --- End of test executable ---

pluginlib_export_plugin_description_file(hardware_interface sim_2d_control.xml)

install(TARGETS sim2d_hardware_interface test_kinematics_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES sim_2d_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(sim2d_hardware_interface)

ament_package()