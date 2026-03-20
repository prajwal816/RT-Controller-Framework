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

/// @file mock_actuator_system.hpp
/// @brief Mock actuator hardware interface for ros2_control.

#ifndef RT_CONTROLLER_FRAMEWORK__HARDWARE_INTERFACE__MOCK_ACTUATOR_SYSTEM_HPP_
#define RT_CONTROLLER_FRAMEWORK__HARDWARE_INTERFACE__MOCK_ACTUATOR_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rt_controller_framework
{
namespace hardware_interface
{

/// @brief Mock actuator system implementing ros2_control SystemInterface.
class MockActuatorSystem : public ::hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockActuatorSystem)

  ::hardware_interface::CallbackReturn on_init(
    const ::hardware_interface::HardwareInfo & info) override;

  ::hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ::hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ::hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<::hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<::hardware_interface::CommandInterface>
  export_command_interfaces() override;

  ::hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  ::hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Pre-allocated state vectors
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Pre-allocated command vectors
  std::vector<double> hw_cmd_positions_;
  std::vector<double> hw_cmd_velocities_;
  std::vector<double> hw_cmd_efforts_;

  // Simulation parameters
  double inertia_{1.0};
  double damping_{0.1};
  double dt_{0.001};
};

}  // namespace hardware_interface
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__HARDWARE_INTERFACE__MOCK_ACTUATOR_SYSTEM_HPP_
