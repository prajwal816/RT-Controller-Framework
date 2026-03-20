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

/// @file rt_thread_config.hpp
/// @brief Real-time thread configuration utilities for Linux RT systems.

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__RT_THREAD_CONFIG_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__RT_THREAD_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rt_controller_framework
{
namespace realtime_utils
{

/// @brief Result of a thread configuration operation.
struct ThreadConfigResult
{
  bool success{false};
  std::string message;
};

/// @brief Set the current thread to SCHED_FIFO real-time scheduling policy.
/// @param priority RT priority (1-99, higher = more urgent).
/// @return ThreadConfigResult with success status and diagnostic message.
ThreadConfigResult set_thread_rt_priority(int priority);

/// @brief Lock all current and future memory pages to prevent page faults.
/// @return ThreadConfigResult with success status and diagnostic message.
ThreadConfigResult lock_memory();

/// @brief Pin the current thread to specific CPU core(s).
/// @param cpu_ids Vector of CPU core indices to pin to (0-indexed).
/// @return ThreadConfigResult with success status and diagnostic message.
ThreadConfigResult set_cpu_affinity(const std::vector<int> & cpu_ids);

/// @brief Apply a complete RT configuration profile to the current thread.
/// @param priority SCHED_FIFO priority (1-99).
/// @param cpu_ids CPU cores to pin to (empty = no affinity change).
/// @return true if ALL configuration steps succeeded.
bool configure_rt_thread(int priority, const std::vector<int> & cpu_ids = {});

/// @brief Get diagnostic information about the RT capabilities of the system.
/// @return Human-readable string with RT kernel detection, capabilities, etc.
std::string get_rt_system_info();

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__RT_THREAD_CONFIG_HPP_
