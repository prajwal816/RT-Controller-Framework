/**
 * @file rt_controller.hpp
 * @brief Real-time PD controller plugin for ros2_control.
 *
 * Implements controller_interface::ControllerInterface to provide a
 * deterministic 1kHz PD controller with lock-free command updates and
 * integrated jitter monitoring.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

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

namespace rt_controller_framework {
namespace controller {

/**
 * @brief Command message for lock-free parameter updates.
 *
 * Non-RT threads push these into the lock-free queue;
 * the RT update() loop pops and applies them.
 */
struct RTCommand {
  std::size_t joint_index{0};
  double target_position{0.0};
  double target_velocity{0.0};
};

/**
 * @brief Real-time PD controller plugin for ros2_control.
 *
 * Features:
 *   - PD position/velocity tracking at 1kHz
 *   - Lock-free command queue for non-RT → RT parameter updates
 *   - Integrated JitterMonitor for loop timing self-diagnostics
 *   - Zero heap allocation in update()
 */
class RTController : public controller_interface::ControllerInterface {
 public:
  RTController();
  ~RTController() override = default;

  /**
   * @brief Declare the command and state interface types.
   */
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  /**
   * @brief Lifecycle: configure controller parameters.
   */
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Real-time update function called at the control frequency.
   *
   * This is the hot path — no dynamic allocation, no blocking calls.
   *
   * @param time Current ROS time.
   * @param period Time since last update.
   * @return controller_interface::return_type::OK on success.
   */
  controller_interface::return_type update(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

 private:
  // Joint configuration
  std::vector<std::string> joint_names_;

  // PD gains (pre-allocated)
  std::vector<double> kp_gains_;
  std::vector<double> kd_gains_;

  // Target positions/velocities (pre-allocated, updated via lock-free queue)
  std::vector<double> target_positions_;
  std::vector<double> target_velocities_;

  // Lock-free command queue (non-RT subscriber → RT update loop)
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
