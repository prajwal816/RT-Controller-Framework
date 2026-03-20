/**
 * @file rt_controller.cpp
 * @brief Implementation of the real-time PD controller plugin.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#include "rt_controller_framework/controller/rt_controller.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#include "controller_interface/helpers.hpp"
#include "rclcpp/qos.hpp"

namespace rt_controller_framework {
namespace controller {

RTController::RTController()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RTController::on_init() {
  try {
    // Declare parameters with defaults
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("kp", std::vector<double>());
    auto_declare<std::vector<double>>("kd", std::vector<double>());
    auto_declare<double>("control_frequency", 1000.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RTController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Read parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints configured!");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto kp = get_node()->get_parameter("kp").as_double_array();
  const auto kd = get_node()->get_parameter("kd").as_double_array();
  control_frequency_ = get_node()->get_parameter("control_frequency").as_double();

  const auto n = joint_names_.size();

  // Pre-allocate gain vectors
  kp_gains_.resize(n, 100.0);  // Default Kp
  kd_gains_.resize(n, 10.0);   // Default Kd

  for (std::size_t i = 0; i < std::min(kp.size(), n); ++i) {
    kp_gains_[i] = kp[i];
  }
  for (std::size_t i = 0; i < std::min(kd.size(), n); ++i) {
    kd_gains_[i] = kd[i];
  }

  // Pre-allocate target arrays
  target_positions_.resize(n, 0.0);
  target_velocities_.resize(n, 0.0);

  // Pre-allocate output message
  state_msg_.data.resize(n * 3, 0.0);  // position, velocity, effort per joint

  // Initialize jitter monitor
  jitter_monitor_ = std::make_unique<realtime_utils::JitterMonitor<1000>>(
      static_cast<uint32_t>(control_frequency_));

  // Create command subscriber (non-RT callback pushes into lock-free queue)
  command_subscriber_ = get_node()->create_subscription<
      std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Non-RT callback — push commands into the lock-free queue
        for (std::size_t i = 0; i < msg->data.size() && i < joint_names_.size();
             ++i) {
          RTCommand cmd;
          cmd.joint_index = i;
          cmd.target_position = msg->data[i];
          cmd.target_velocity = 0.0;
          command_queue_.try_push(cmd);  // Drop if full (RT-safe)
        }
      });

  // Create state publisher
  state_publisher_ = get_node()->create_publisher<
      std_msgs::msg::Float64MultiArray>(
      "~/state", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured RTController with %zu joints at %.0f Hz",
              n, control_frequency_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RTController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Initialize target positions from current state
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& pos_interface = state_interfaces_[i * 3];  // position
    target_positions_[i] = pos_interface.get_value();
  }

  // Reset jitter monitor
  jitter_monitor_->reset();
  first_update_ = true;

  RCLCPP_INFO(get_node()->get_logger(), "RTController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RTController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Log final jitter statistics
  RCLCPP_INFO(get_node()->get_logger(), "\n%s",
              jitter_monitor_->format_summary().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RTController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
RTController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::return_type RTController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // ──────────────────────────────────────────────────────────────────────
  // HOT PATH — No dynamic allocation, no blocking, no exceptions
  // ──────────────────────────────────────────────────────────────────────

  // Measure jitter
  auto now = std::chrono::steady_clock::now();
  if (!first_update_) {
    auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
                     now - last_update_time_)
                     .count();
    jitter_monitor_->record_cycle(dt_us);
  }
  first_update_ = false;
  last_update_time_ = now;

  // Drain command queue (non-blocking)
  while (auto cmd = command_queue_.try_pop()) {
    if (cmd->joint_index < target_positions_.size()) {
      target_positions_[cmd->joint_index] = cmd->target_position;
      target_velocities_[cmd->joint_index] = cmd->target_velocity;
    }
  }

  // PD control for each joint
  const auto n = joint_names_.size();
  for (std::size_t i = 0; i < n; ++i) {
    // Read current state (no allocation — direct pointer access)
    const double current_pos = state_interfaces_[i * 3 + 0].get_value();
    const double current_vel = state_interfaces_[i * 3 + 1].get_value();

    // Compute PD output
    const double pos_error = target_positions_[i] - current_pos;
    const double vel_error = target_velocities_[i] - current_vel;
    const double effort = kp_gains_[i] * pos_error + kd_gains_[i] * vel_error;

    // Write commands (no allocation — direct pointer access)
    command_interfaces_[i * 3 + 0].set_value(target_positions_[i]);
    command_interfaces_[i * 3 + 1].set_value(target_velocities_[i]);
    command_interfaces_[i * 3 + 2].set_value(effort);

    // Fill pre-allocated state message
    state_msg_.data[i * 3 + 0] = current_pos;
    state_msg_.data[i * 3 + 1] = current_vel;
    state_msg_.data[i * 3 + 2] = effort;
  }

  // Publish state (uses pre-allocated message)
  state_publisher_->publish(state_msg_);

  return controller_interface::return_type::OK;
}

}  // namespace controller
}  // namespace rt_controller_framework

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rt_controller_framework::controller::RTController,
    controller_interface::ControllerInterface)
