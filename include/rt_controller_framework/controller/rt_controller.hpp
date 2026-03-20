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

/// @file rt_controller.hpp
/// @brief Real-time PD controller plugin for ros2_control.

#ifndef RT_CONTROLLER_FRAMEWORK__CONTROLLER__RT_CONTROLLER_HPP_
#define RT_CONTROLLER_FRAMEWORK__CONTROLLER__RT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"
#include "rt_controller_framework/realtime_utils/lock_free_queue.hpp"

namespace rt_controller_framework
{
namespace controller
{

/// @brief Command message for lock-free parameter updates.
struct RTCommand
{
  std::size_t joint_index{0};
  double target_position{0.0};
  double target_velocity{0.0};
};

/// @brief Real-time PD controller plugin for ros2_control.
class RTController : public controller_interface::ControllerInterface
{
public:
  RTController();
  ~RTController() override = default;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Joint configuration
  std::vector<std::string> joint_names_;

  // PD gains (pre-allocated)
  std::vector<double> kp_gains_;
  std::vector<double> kd_gains_;

  // Target positions/velocities (pre-allocated, updated via lock-free queue)
  std::vector<double> target_positions_;
  std::vector<double> target_velocities_;

  // Lock-free command queue (non-RT subscriber -> RT update loop)
  realtime_utils::LockFreeQueue<RTCommand, 64> command_queue_;

  // Jitter monitoring
  std::unique_ptr<realtime_utils::JitterMonitor<1000>> jitter_monitor_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    command_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    state_publisher_;

  // Pre-allocated output message
  std_msgs::msg::Float64MultiArray state_msg_;

  // Timing for jitter measurement
  std::chrono::steady_clock::time_point last_update_time_;
  bool first_update_{true};

  // Control frequency
  double control_frequency_{1000.0};
};

}  // namespace controller
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__CONTROLLER__RT_CONTROLLER_HPP_
