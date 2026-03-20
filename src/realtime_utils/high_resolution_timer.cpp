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

/// @file high_resolution_timer.cpp
/// @brief Implementation of the high-resolution periodic timer.

#include "rt_controller_framework/realtime_utils/high_resolution_timer.hpp"

#include <thread>

namespace rt_controller_framework
{
namespace realtime_utils
{

HighResolutionTimer::HighResolutionTimer(uint32_t frequency_hz)
: frequency_hz_(frequency_hz),
  period_(std::chrono::duration_cast<duration>(
    std::chrono::microseconds(1000000 / frequency_hz))),
  start_time_(clock::now()),
  next_wakeup_(clock::now()),
  last_wakeup_(clock::now())
{
}

void HighResolutionTimer::start() noexcept
{
  start_time_ = clock::now();
  next_wakeup_ = start_time_ + period_;
  last_wakeup_ = start_time_;
  cycle_count_ = 0;
  actual_period_us_ = 0;
}

void HighResolutionTimer::wait_for_next_cycle() noexcept
{
  // Sleep until the next scheduled wakeup
  std::this_thread::sleep_until(next_wakeup_);

  // Measure actual period
  auto now = clock::now();
  auto actual = std::chrono::duration_cast<std::chrono::microseconds>(
    now - last_wakeup_);
  actual_period_us_ = actual.count();
  last_wakeup_ = now;

  // Schedule next cycle — advance from the ideal wakeup, not 'now',
  // to prevent accumulating drift. But if we overran significantly
  // (> 2× period), reset to prevent cascading delays.
  next_wakeup_ += period_;

  if (now > next_wakeup_ + period_) {
    // Overrun recovery: reset the wakeup target
    next_wakeup_ = now + period_;
  }

  ++cycle_count_;
}

int64_t HighResolutionTimer::get_actual_period_us() const noexcept
{
  return actual_period_us_;
}

HighResolutionTimer::duration
HighResolutionTimer::get_elapsed() const noexcept
{
  return clock::now() - start_time_;
}

uint64_t HighResolutionTimer::get_cycle_count() const noexcept
{
  return cycle_count_;
}

HighResolutionTimer::duration
HighResolutionTimer::get_period() const noexcept
{
  return period_;
}

uint32_t HighResolutionTimer::get_frequency_hz() const noexcept
{
  return frequency_hz_;
}

}  // namespace realtime_utils
}  // namespace rt_controller_framework
