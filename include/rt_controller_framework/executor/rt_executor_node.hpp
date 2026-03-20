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

/// @file rt_executor_node.hpp
/// @brief Real-time optimized executor node for deterministic scheduling.

#ifndef RT_CONTROLLER_FRAMEWORK__EXECUTOR__RT_EXECUTOR_NODE_HPP_
#define RT_CONTROLLER_FRAMEWORK__EXECUTOR__RT_EXECUTOR_NODE_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rt_controller_framework/realtime_utils/high_resolution_timer.hpp"
#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"

namespace rt_controller_framework
{
namespace executor
{

/// @brief RT Executor Node -- standalone demo of deterministic 1kHz loop.
class RTExecutorNode : public rclcpp::Node
{
public:
  /// @brief Construct the RT executor node.
  /// @param frequency_hz Control loop frequency (default: 1000).
  /// @param num_cycles Number of cycles to run (0 = infinite).
  explicit RTExecutorNode(
    uint32_t frequency_hz = 1000,
    uint64_t num_cycles = 5000);

  ~RTExecutorNode() override;

  /// @brief Start the RT loop on a dedicated thread.
  void start();

  /// @brief Stop the RT loop and join the thread.
  void stop();

  /// @brief Check if the RT loop is currently running.
  [[nodiscard]] bool is_running() const;

  /// @brief Get the jitter statistics from the completed run.
  [[nodiscard]] realtime_utils::JitterMonitor<1000>::Statistics
  get_statistics() const;

private:
  /// @brief The RT loop function -- runs on a dedicated thread.
  void rt_loop();

  /// @brief Publish metrics via ROS topic (called from timer callback).
  void publish_metrics();

  uint32_t frequency_hz_;
  uint64_t num_cycles_;
  std::atomic<bool> running_{false};
  std::thread rt_thread_;

  realtime_utils::HighResolutionTimer timer_;
  realtime_utils::JitterMonitor<1000> jitter_monitor_;

  // ROS publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    metrics_publisher_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;

  // Pre-allocated metrics message
  std_msgs::msg::Float64MultiArray metrics_msg_;
};

}  // namespace executor
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__EXECUTOR__RT_EXECUTOR_NODE_HPP_
