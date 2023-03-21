// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "blv_diffbot_hardware/blv_diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blv_diffbot_hardware
{
CallbackReturn BlvDiffbotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  device_name_ = "/dev/ttyACM0";
  device_name_ = info_.hardware_parameters["device_name"];
  gear_ratio_ = stod(info_.hardware_parameters["gear_ratio"]);
  encoder_resolution_ = stod(info_.hardware_parameters["encoder_resolution"]);
  
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // BlvDiffbotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BlvDiffbotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BlvDiffbotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BlvDiffbotSystemHardware"),
        "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BlvDiffbotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BlvDiffbotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BlvDiffbotSystemHardware"),
        "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  serial_port_ = std::make_shared<BlvComunicator>();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BlvDiffbotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BlvDiffbotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn BlvDiffbotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "Starting ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("BlvDiffbotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  
  if (serial_port_->openDevice(device_name_) != BlvComunicator::return_type::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn BlvDiffbotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "Stopping ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("BlvDiffbotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  
  if (serial_port_->is_open) {
    serial_port_->closeDevice();
  }

  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BlvDiffbotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "Reading...");

  int motor_direction;
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    if (i % 2 == 0)
    {
      motor_direction = 1;
    }
    else
    {
      motor_direction = -1;
    }

    int rpm = 0;
    serial_port_->readRpm(i+1, &rpm);
    hw_positions_[i] += static_cast<double>(motor_direction * rpm) * 2.0 * M_PI / gear_ratio_ / 60.0 * period.seconds();
    hw_velocities_[i] = static_cast<double>(motor_direction * rpm) * 2.0 * M_PI / gear_ratio_ / 60.0;

    int torque = 0;
    serial_port_->readTorque(i+1, &torque);
    hw_efforts_[i] = static_cast<double>(motor_direction * torque) / 1000.0 * BlvComunicator::BLV_RATED_TORQUE * gear_ratio_;

    RCLCPP_INFO(
      rclcpp::get_logger("BlvDiffbotSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BlvDiffbotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "Writing...");

  int motor_direction;
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (i % 2 == 0)
    {
      motor_direction = 1;
    }
    else
    {
      motor_direction = -1;
    }

    // Generate the motor command message
    int rpm = motor_direction * static_cast<int>(hw_commands_[i] * 60.0 * gear_ratio_ / (2.0 * M_PI));

    std::cout << "rpm[" << i << "] = " << rpm << std::endl;
    serial_port_->writeRpm(i+1, rpm);
    
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("BlvDiffbotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("BlvDiffbotSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace blv_diffbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  blv_diffbot_hardware::BlvDiffbotSystemHardware, hardware_interface::SystemInterface)
