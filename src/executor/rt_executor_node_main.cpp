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

/// @file rt_executor_node_main.cpp
/// @brief Entry point for the standalone RT executor node.

#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rt_controller_framework/executor/rt_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the RT executor node with 1kHz, 10000 cycles (~10 seconds)
  auto node = std::make_shared<rt_controller_framework::executor::RTExecutorNode>(
    1000, 10000);

  // Start the RT loop on a dedicated thread
  node->start();

  // Spin the node for ROS callbacks (metrics publishing)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && node->is_running()) {
    executor.spin_some(std::chrono::milliseconds(100));
  }

  // Final spin to flush pending callbacks
  executor.spin_some(std::chrono::milliseconds(100));

  node->stop();

  // Print final stats
  auto stats = node->get_statistics();
  RCLCPP_INFO(
    node->get_logger(),
    "=== FINAL RESULTS ===\n"
    "  Total Cycles:  %" PRIu64 "\n"
    "  Mean Jitter:   %.2f us\n"
    "  Max Jitter:    %.2f us\n"
    "  Stddev Jitter: %.2f us\n"
    "  Mean Period:   %.2f us\n"
    "  Overruns:      %" PRIu64,
    stats.total_cycles,
    stats.mean_jitter_us,
    stats.max_jitter_us,
    stats.stddev_jitter_us,
    stats.mean_period_us,
    stats.overrun_count);

  rclcpp::shutdown();
  return 0;
}
