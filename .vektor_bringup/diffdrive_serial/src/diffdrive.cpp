#include "diffdrive_serial/diffdrive.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"

// #include <algorithm>

namespace diffdrive_serial
{

DiffDriveSerial::DiffDriveSerial()
{
}

DiffDriveSerial::~DiffDriveSerial()
{
}

hardware_interface::CallbackReturn
DiffDriveSerial::on_init(const hardware_interface::HardwareInfo & info)
{
  if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  hw_positions_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSerial"),
        "Joint '%s' has '%zu' command interfaces found. 1 expected",
        joint.name.c_str(),
        joint.command_interfaces.size()
      );
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSerial"),
        "Joint '%s' have '%s' command interfaces found. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY
      );
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSerial"),
        "Joint '%s' has '%zu' state interface. 2 expected.",
        joint.name.c_str(),
        joint.state_interfaces.size()
      );
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSerial"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION
      );
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSerial"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY
      );
      return hardware_interface::CallbackReturn::ERROR;
    }     
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
DiffDriveSerial::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]
    ));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]
    ));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveSerial::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]
    ));
  }
  return command_interfaces;
}

// hardware_interface::CallbackReturn
// DiffDriveSerial::on_configure(const rclcpp_lifecycle::State & previous_state)
// {
//   std::fill(hw_gpio_in_.begin(), hw_gpio_in_.end(), 0);
//   std::fill(hw_gpio_out_.begin(), hw_gpio_out_.end(), 0);

//   RCLCPP_INFO(
//     rclcpp::get_logger("hardware_interface"),
//     "Sucess on configure"
//   );

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

}  // namespace diffdrive_serial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_serial::DiffDriveSerial, hardware_interface::SystemInterface
)
