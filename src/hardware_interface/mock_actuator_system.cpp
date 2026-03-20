// Copyright 2024 RT Controller Framework Contributors
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

/// @file mock_actuator_system.cpp
/// @brief Implementation of the mock actuator hardware interface.

#include "rt_controller_framework/hardware_interface/mock_actuator_system.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rt_controller_framework
{
namespace hardware_interface
{

::hardware_interface::CallbackReturn MockActuatorSystem::on_init(
  const ::hardware_interface::HardwareInfo & info)
{
  if (::hardware_interface::SystemInterface::on_init(info) !=
    ::hardware_interface::CallbackReturn::SUCCESS)
  {
    return ::hardware_interface::CallbackReturn::ERROR;
  }

  // Read simulation parameters from hardware info
  if (info_.hardware_parameters.count("inertia")) {
    inertia_ = std::stod(info_.hardware_parameters.at("inertia"));
  }
  if (info_.hardware_parameters.count("damping")) {
    damping_ = std::stod(info_.hardware_parameters.at("damping"));
  }
  if (info_.hardware_parameters.count("control_period")) {
    dt_ = std::stod(info_.hardware_parameters.at("control_period"));
  }

  const auto num_joints = info_.joints.size();

  // Pre-allocate all vectors (this is the ONLY allocation point)
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_cmd_positions_.resize(num_joints, 0.0);
  hw_cmd_velocities_.resize(num_joints, 0.0);
  hw_cmd_efforts_.resize(num_joints, 0.0);

  // Set initial positions from URDF parameters if available
  for (std::size_t i = 0; i < num_joints; ++i) {
    const auto & joint = info_.joints[i];
    if (joint.parameters.count("initial_position")) {
      hw_positions_[i] = std::stod(joint.parameters.at("initial_position"));
      hw_cmd_positions_[i] = hw_positions_[i];
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MockActuatorSystem"),
    "Initialized %zu joints (inertia=%.3f, damping=%.3f, dt=%.6f)",
    num_joints, inertia_, damping_, dt_);

  return ::hardware_interface::CallbackReturn::SUCCESS;
}

::hardware_interface::CallbackReturn MockActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MockActuatorSystem"), "Configuring...");

  // Reset states to initial values
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_cmd_velocities_[i] = 0.0;
    hw_cmd_efforts_[i] = 0.0;
  }

  return ::hardware_interface::CallbackReturn::SUCCESS;
}

::hardware_interface::CallbackReturn MockActuatorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MockActuatorSystem"), "Activating...");

  // Sync command positions to current positions
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_cmd_positions_[i] = hw_positions_[i];
    hw_cmd_velocities_[i] = 0.0;
    hw_cmd_efforts_[i] = 0.0;
  }

  return ::hardware_interface::CallbackReturn::SUCCESS;
}

::hardware_interface::CallbackReturn MockActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MockActuatorSystem"), "Deactivating...");
  return ::hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<::hardware_interface::StateInterface>
MockActuatorSystem::export_state_interfaces()
{
  std::vector<::hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      ::hardware_interface::StateInterface(
        info_.joints[i].name,
        ::hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(
      ::hardware_interface::StateInterface(
        info_.joints[i].name,
        ::hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
    state_interfaces.emplace_back(
      ::hardware_interface::StateInterface(
        info_.joints[i].name,
        ::hardware_interface::HW_IF_EFFORT,
        &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<::hardware_interface::CommandInterface>
MockActuatorSystem::export_command_interfaces()
{
  std::vector<::hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 3);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      ::hardware_interface::CommandInterface(
        info_.joints[i].name,
        ::hardware_interface::HW_IF_POSITION,
        &hw_cmd_positions_[i]));
    command_interfaces.emplace_back(
      ::hardware_interface::CommandInterface(
        info_.joints[i].name,
        ::hardware_interface::HW_IF_VELOCITY,
        &hw_cmd_velocities_[i]));
    command_interfaces.emplace_back(
      ::hardware_interface::CommandInterface(
        info_.joints[i].name,
        ::hardware_interface::HW_IF_EFFORT,
        &hw_cmd_efforts_[i]));
  }

  return command_interfaces;
}

::hardware_interface::return_type MockActuatorSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // For the mock, the states are already updated by write().
  return ::hardware_interface::return_type::OK;
}

::hardware_interface::return_type MockActuatorSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Simulate actuator dynamics using Euler integration
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    // Compute net effort (commanded effort + PD position tracking)
    double effort = hw_cmd_efforts_[i];

    // If position command is significantly different, add a spring force
    double position_error = hw_cmd_positions_[i] - hw_positions_[i];
    double velocity_error = hw_cmd_velocities_[i] - hw_velocities_[i];

    // Simple PD spring-damper model for position tracking
    const double kp = 100.0;
    const double kd = 10.0;
    effort += kp * position_error + kd * velocity_error;

    // Euler integration
    double acceleration = (effort - damping_ * hw_velocities_[i]) / inertia_;
    hw_velocities_[i] += acceleration * dt_;
    hw_positions_[i] += hw_velocities_[i] * dt_;

    // Store the applied effort as state
    hw_efforts_[i] = effort;
  }

  return ::hardware_interface::return_type::OK;
}

}  // namespace hardware_interface
}  // namespace rt_controller_framework

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rt_controller_framework::hardware_interface::MockActuatorSystem,
  ::hardware_interface::SystemInterface)
