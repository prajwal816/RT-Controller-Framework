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

/// @file high_resolution_timer.hpp
/// @brief High-resolution periodic timer for deterministic control loops.

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__HIGH_RESOLUTION_TIMER_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__HIGH_RESOLUTION_TIMER_HPP_

#include <chrono>
#include <cstdint>

namespace rt_controller_framework
{
namespace realtime_utils
{

/// @brief Precise periodic timer using steady_clock and sleep_until.
class HighResolutionTimer
{
public:
  using clock = std::chrono::steady_clock;
  using time_point = clock::time_point;
  using duration = clock::duration;

  /// @brief Construct timer with desired frequency.
  /// @param frequency_hz Control loop frequency (e.g. 1000 for 1kHz).
  explicit HighResolutionTimer(uint32_t frequency_hz);

  /// @brief Start (or restart) the timer.
  void start() noexcept;

  /// @brief Sleep until the next cycle boundary.
  void wait_for_next_cycle() noexcept;

  /// @brief Get the measured period of the last completed cycle.
  /// @return Actual period in microseconds.
  [[nodiscard]] int64_t get_actual_period_us() const noexcept;

  /// @brief Get total elapsed time since start().
  /// @return Elapsed duration.
  [[nodiscard]] duration get_elapsed() const noexcept;

  /// @brief Get the number of completed cycles.
  [[nodiscard]] uint64_t get_cycle_count() const noexcept;

  /// @brief Get the configured period.
  [[nodiscard]] duration get_period() const noexcept;

  /// @brief Get the configured frequency.
  [[nodiscard]] uint32_t get_frequency_hz() const noexcept;

private:
  uint32_t frequency_hz_;
  duration period_;
  time_point start_time_;
  time_point next_wakeup_;
  time_point last_wakeup_;
  int64_t actual_period_us_{0};
  uint64_t cycle_count_{0};
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__HIGH_RESOLUTION_TIMER_HPP_
