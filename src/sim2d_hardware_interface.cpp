#include "sim2d_hardware_interface/sim2d_hardware_interface.hpp"

#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace sim2d_hardware_interface
{

hardware_interface::CallbackReturn Sim2DHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "Configuring...");

  // Parse parameters for the kinematic solver from <hardware> tag
  try {
    icr_offset_x_ = std::stod(info_.hardware_parameters.at("x_icr"));
    RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "x_icr: %f", icr_offset_x_);
  } catch (const std::out_of_range & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("Sim2DHardwareInterface"), "Required parameter not found: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("Sim2DHardwareInterface"), "Invalid argument for parameter: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "Parsing joints...");

  for (const auto & joint : info_.joints) {
      Wheel wheel;
      try {
          // These parameters are mandatory for every wheel joint
          // wheel.name = joint.name;  // Wheel struct doesn't have name
          wheel.position_x = std::stod(joint.parameters.at("x"));
          wheel.position_y = std::stod(joint.parameters.at("y"));
          wheel.radius = std::stod(joint.parameters.at("radius"));
          wheel.mounting_angle = std::stod(joint.parameters.at("mount_angle"));
          // wheel.steerable = (joint.parameters.at("steerable") == "true");
          // wheel.zero_command_behavior = joint.parameters.at("zero_command_behavior");
          wheel.alpha_slip = std::stod(joint.parameters.at("alpha"));
          wheel.beta1_roc = std::stod(joint.parameters.at("beta1"));
          wheel.beta2_roc = std::stod(joint.parameters.at("beta2"));
          
          wheels_.push_back(wheel);
          RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "Added wheel '%s' at (%f, %f)", 
            joint.name.c_str(), wheel.position_x, wheel.position_y);

      } catch (const std::out_of_range & ex) {
          RCLCPP_FATAL(rclcpp::get_logger("Sim2DHardwareInterface"), "Joint '%s' missing parameter: %s", joint.name.c_str(), ex.what());
          return hardware_interface::CallbackReturn::ERROR;
      } catch (const std::invalid_argument & ex) {
          RCLCPP_FATAL(rclcpp::get_logger("Sim2DHardwareInterface"), "Joint '%s' has invalid argument: %s", joint.name.c_str(), ex.what());
          return hardware_interface::CallbackReturn::ERROR;
      }
  }

  if (wheels_.empty()) {
      RCLCPP_FATAL(rclcpp::get_logger("Sim2DHardwareInterface"), "No wheels configured. Make sure to add wheel parameters to your URDF joints.");
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Create a ROS 2 node for publishers
  // We use a temporary node to get the logger and parameters
  auto node = std::make_shared<rclcpp::Node>("sim2d_hardware_interface_node");
  odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  node_ = node; // Store the node

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "Initialization successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Sim2DHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Sim2DHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Check if the joint is for steering (position) or driving (velocity)
    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    else // Assuming velocity otherwise
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn Sim2DHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands and states
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_states_positions_[i] = 0.0;
    hw_states_velocities_[i] = 0.0;
    hw_commands_positions_[i] = 0.0;
    hw_commands_velocities_[i] = 0.0;
  }

  // Initialize the KinematicSolver with parameters from URDF
  solver_ = std::make_unique<KinematicSolver>(wheels_, icr_offset_x_);

  RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "Activation successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Sim2DHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Sim2DHardwareInterface"), "Deactivation successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Sim2DHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // In our simulation, the "reading" is just updating the state based on the last command.
  // The main logic is in write(), where we calculate the new state.
  // Here, we just reflect the latest calculated state for ros2_control.
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      // The velocity state is assumed to be the same as the command for simplicity
      // in the context of a simulator where state follows command instantly.
      hw_states_velocities_[i] = hw_commands_velocities_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Sim2DHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!solver_)
  {
    return hardware_interface::return_type::OK; // Solver not initialized yet
  }

  // Collect wheel speeds and steering angles from the hardware commands
  std::vector<double> wheel_speeds;
  std::vector<double> steering_angles;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
          wheel_speeds.push_back(hw_commands_velocities_[i]);
      } else if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
          steering_angles.push_back(hw_commands_positions_[i]);
      }
  }

  // Get chassis velocity from the kinematic solver
  double vx = 0.0, vy = 0.0, omega = 0.0;
  solver_->solve(wheel_speeds, steering_angles, vx, vy, omega);

  // Integrate the robot's pose
  double dt = period.seconds();
  double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
  double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
  double delta_theta = omega * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // Update joint positions for visualization in RViz
  for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
          // For driving wheels, integrate position from velocity
          hw_states_positions_[i] += hw_commands_velocities_[i] * dt;
      } else if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
          // For steering wheels, position state is the same as the command
          hw_states_positions_[i] = hw_commands_positions_[i];
      }
  }

  // Publish odometry and TF
  rclcpp::Time now = time;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);

  // TF
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = "odom";
  transform.child_frame_id = base_frame_id_;
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(transform);

  // Odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = base_frame_id_;
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = transform.transform.rotation;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;
  odom_publisher_->publish(odom_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace sim2d_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sim2d_hardware_interface::Sim2DHardwareInterface,
  hardware_interface::SystemInterface)
