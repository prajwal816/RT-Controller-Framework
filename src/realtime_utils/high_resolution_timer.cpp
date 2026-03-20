/**
 * @file high_resolution_timer.cpp
 * @brief Implementation of the high-resolution periodic timer.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#include "rt_controller_framework/realtime_utils/high_resolution_timer.hpp"

#include <thread>

namespace rt_controller_framework {
namespace realtime_utils {

HighResolutionTimer::HighResolutionTimer(uint32_t frequency_hz)
    : frequency_hz_(frequency_hz),
      period_(std::chrono::duration_cast<Duration>(
          std::chrono::microseconds(1000000 / frequency_hz))),
      start_time_(Clock::now()),
      next_wakeup_(Clock::now()),
      last_wakeup_(Clock::now()) {}

void HighResolutionTimer::start() {
  start_time_ = Clock::now();
  next_wakeup_ = start_time_ + period_;
  last_wakeup_ = start_time_;
  cycle_count_ = 0;
  actual_period_us_ = 0;
  overrun_ = false;
}

void HighResolutionTimer::wait_for_next_cycle() {
  // Sleep until the next target wakeup time
  std::this_thread::sleep_until(next_wakeup_);

  // Measure actual period
  auto now = Clock::now();
  actual_period_us_ =
      std::chrono::duration_cast<std::chrono::microseconds>(now - last_wakeup_)
          .count();

  // Check for deadline overrun (> 1.5× period)
  int64_t threshold = get_period_us() + (get_period_us() / 2);
  overrun_ = (actual_period_us_ > threshold);

  // Advance state
  last_wakeup_ = now;
  next_wakeup_ += period_;
  ++cycle_count_;

  // If we fell behind, reset the wakeup target to prevent cascading delays
  if (next_wakeup_ < now) {
    next_wakeup_ = now + period_;
  }
}

int64_t HighResolutionTimer::get_period_us() const {
  return std::chrono::duration_cast<std::chrono::microseconds>(period_).count();
}

int64_t HighResolutionTimer::get_actual_period_us() const {
  return actual_period_us_;
}

uint32_t HighResolutionTimer::get_frequency_hz() const {
  return frequency_hz_;
}

uint64_t HighResolutionTimer::get_cycle_count() const {
  return cycle_count_;
}

int64_t HighResolutionTimer::get_elapsed_us() const {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             Clock::now() - start_time_)
      .count();
}

bool HighResolutionTimer::has_overrun() const {
  return overrun_;
}

}  // namespace realtime_utils
}  // namespace rt_controller_framework
