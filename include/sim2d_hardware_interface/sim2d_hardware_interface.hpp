#ifndef SIM2D_HARDWARE_INTERFACE__SIM2D_HARDWARE_INTERFACE_HPP_
#define SIM2D_HARDWARE_INTERFACE__SIM2D_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/component_parser.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

namespace sim2d_hardware_interface
{

class Sim2DHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
};

}  // namespace sim2d_hardware_interface

#endif  // SIM2D_HARDWARE_INTERFACE__SIM2D_HARDWARE_INTERFACE_HPP_
