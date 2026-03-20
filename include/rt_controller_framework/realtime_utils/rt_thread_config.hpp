/**
 * @file rt_thread_config.hpp
 * @brief Real-time thread configuration utilities for Linux RT systems.
 *
 * Provides functions for setting SCHED_FIFO priority, locking memory
 * pages, and pinning threads to specific CPU cores. These are essential
 * for achieving deterministic timing in hard real-time control loops.
 *
 * On non-Linux platforms, functions are no-ops that return gracefully.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__RT_THREAD_CONFIG_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__RT_THREAD_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rt_controller_framework {
namespace realtime_utils {

/**
 * @brief Result of a thread configuration operation.
 */
struct ThreadConfigResult {
  bool success{false};
  std::string message;
};

/**
 * @brief Set the current thread to SCHED_FIFO real-time scheduling policy.
 *
 * @param priority RT priority (1-99, higher = more urgent). Typical values:
 *                 - 80 for control loops
 *                 - 90 for safety monitors
 *                 - 50 for non-critical RT tasks
 * @return ThreadConfigResult with success status and diagnostic message.
 *
 * @note Requires root privileges or CAP_SYS_NICE capability.
 *
 * Usage:
 * @code
 *   auto result = set_thread_rt_priority(80);
 *   if (!result.success) {
 *     RCLCPP_WARN(logger, "RT priority: %s", result.message.c_str());
 *   }
 * @endcode
 */
ThreadConfigResult set_thread_rt_priority(int priority);

/**
 * @brief Lock all current and future memory pages to prevent page faults.
 *
 * Calls mlockall(MCL_CURRENT | MCL_FUTURE) to prevent the OS from
 * swapping any of the process's memory pages to disk. This eliminates
 * a major source of non-deterministic latency in RT systems.
 *
 * @return ThreadConfigResult with success status and diagnostic message.
 *
 * @note Requires root privileges or CAP_IPC_LOCK capability.
 */
ThreadConfigResult lock_memory();

/**
 * @brief Pin the current thread to specific CPU core(s).
 *
 * Sets the CPU affinity mask so the thread only runs on the specified
 * cores. Use with isolated cores (isolcpus kernel parameter) for best
 * determinism.
 *
 * @param cpu_ids Vector of CPU core indices to pin to (0-indexed).
 * @return ThreadConfigResult with success status and diagnostic message.
 *
 * Usage:
 * @code
 *   // Pin to core 2
 *   auto result = set_cpu_affinity({2});
 *   // Pin to cores 2 and 3
 *   auto result = set_cpu_affinity({2, 3});
 * @endcode
 */
ThreadConfigResult set_cpu_affinity(const std::vector<int>& cpu_ids);

/**
 * @brief Apply a complete RT configuration profile to the current thread.
 *
 * Convenience function that applies priority, memory locking, and CPU
 * affinity in one call. Logs results for each step.
 *
 * @param priority SCHED_FIFO priority (1-99).
 * @param cpu_ids  CPU cores to pin to (empty = no affinity change).
 * @return true if ALL configuration steps succeeded.
 */
bool configure_rt_thread(int priority, const std::vector<int>& cpu_ids = {});

/**
 * @brief Get diagnostic information about the RT capabilities of the system.
 * @return Human-readable string with RT kernel detection, capabilities, etc.
 */
std::string get_rt_system_info();

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__RT_THREAD_CONFIG_HPP_
