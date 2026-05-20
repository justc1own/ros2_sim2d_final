#ifndef SIM2D_HARDWARE_INTERFACE__SIM2D_HARDWARE_INTERFACE_HPP_
#define SIM2D_HARDWARE_INTERFACE__SIM2D_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
// #include "rclcpp_lifecycle/node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sim2d_hardware_interface/kinematics_solver.hpp"

namespace sim2d_hardware_interface
{

class Sim2DHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Sim2DHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  // The kinematic solver
  std::unique_ptr<KinematicSolver> solver_;

  // Parameters for the kinematic solver
  std::vector<Wheel> wheels_;
  double icr_offset_x_ = 0.0;
  std::string base_frame_id_ = "base_link";

  // Robot's pose
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  // ROS 2 publishers
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

}  // namespace sim2d_hardware_interface

#endif  // SIM2D_HARDWARE_INTERFACE__SIM2D_HARDWARE_INTERFACE_HPP_

