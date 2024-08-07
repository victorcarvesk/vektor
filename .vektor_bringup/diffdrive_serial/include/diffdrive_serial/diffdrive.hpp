#pragma once

#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffdrive_serial/visibility_control.h"

namespace diffdrive_serial
{

class DiffDriveSerial : public hardware_interface::SystemInterface
{
public:
  DiffDriveSerial();
  
  /* Hardware Interface */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period);
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /* Lifecycle Node Interface */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  // hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  // hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  // hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  virtual ~DiffDriveSerial();

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace diffdrive_serial
