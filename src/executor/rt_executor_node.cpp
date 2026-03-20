/**
 * @file rt_executor_node.cpp
 * @brief Implementation of the RT executor node.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#include "rt_controller_framework/executor/rt_executor_node.hpp"

#include <chrono>
#include <functional>

namespace rt_controller_framework {
namespace executor {

RTExecutorNode::RTExecutorNode(uint32_t frequency_hz, uint64_t num_cycles)
    : Node("rt_executor_node"),
      frequency_hz_(frequency_hz),
      num_cycles_(num_cycles),
      timer_(frequency_hz),
      jitter_monitor_(frequency_hz) {
  // Pre-allocate metrics message: [mean_jitter, max_jitter, stddev, mean_period,
  //                                 cycle_count, overrun_count]
  metrics_msg_.data.resize(6, 0.0);

  // Create metrics publisher
  metrics_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "rt_metrics", rclcpp::SystemDefaultsQoS());

  // Create a wall timer to periodically publish metrics (1 Hz)
  metrics_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RTExecutorNode::publish_metrics, this));

  RCLCPP_INFO(this->get_logger(),
              "RTExecutorNode created: %u Hz, %lu cycles",
              frequency_hz_, static_cast<unsigned long>(num_cycles_));
}

RTExecutorNode::~RTExecutorNode() {
  stop();
}

void RTExecutorNode::start() {
  if (running_.load()) {
    RCLCPP_WARN(this->get_logger(), "RT loop already running");
    return;
  }

  running_.store(true);
  jitter_monitor_.reset();
  rt_thread_ = std::thread(&RTExecutorNode::rt_loop, this);

  RCLCPP_INFO(this->get_logger(), "RT loop started on dedicated thread");
}

void RTExecutorNode::stop() {
  running_.store(false);
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
}

bool RTExecutorNode::is_running() const {
  return running_.load();
}

realtime_utils::JitterMonitor<1000>::Statistics
RTExecutorNode::get_statistics() const {
  return jitter_monitor_.get_statistics();
}

void RTExecutorNode::rt_loop() {
  // ──────────────────────────────────────────────────────────────────────
  // REAL-TIME LOOP — No dynamic allocation, no blocking I/O
  // ──────────────────────────────────────────────────────────────────────

  timer_.start();
  uint64_t cycle = 0;

  while (running_.load(std::memory_order_relaxed)) {
    // Wait for next cycle boundary (sleep_until for precision)
    timer_.wait_for_next_cycle();

    // Record jitter
    jitter_monitor_.record_cycle(timer_.get_actual_period_us());

    // Simulated RT workload (minimal to measure pure timing jitter)
    // In a real system, this would be the control computation.
    volatile double dummy = 0.0;
    for (int i = 0; i < 10; ++i) {
      dummy += static_cast<double>(i) * 0.001;
    }
    (void)dummy;

    ++cycle;

    // Check termination
    if (num_cycles_ > 0 && cycle >= num_cycles_) {
      break;
    }
  }

  running_.store(false);

  // Log final statistics
  RCLCPP_INFO(this->get_logger(), "\n%s",
              jitter_monitor_.format_summary().c_str());
}

void RTExecutorNode::publish_metrics() {
  if (!running_.load()) return;

  auto stats = jitter_monitor_.get_statistics();

  // Fill pre-allocated message (no allocation)
  metrics_msg_.data[0] = stats.mean_jitter_us;
  metrics_msg_.data[1] = stats.max_jitter_us;
  metrics_msg_.data[2] = stats.stddev_jitter_us;
  metrics_msg_.data[3] = stats.mean_period_us;
  metrics_msg_.data[4] = static_cast<double>(stats.total_cycles);
  metrics_msg_.data[5] = static_cast<double>(stats.overrun_count);

  metrics_publisher_->publish(metrics_msg_);
}

}  // namespace executor
}  // namespace rt_controller_framework
