/**
 * @file high_resolution_timer.hpp
 * @brief High-resolution periodic timer for deterministic real-time loops.
 *
 * Uses std::chrono::steady_clock for monotonic, high-resolution timing.
 * Provides sleep_until-based periodic wakeups to achieve 1kHz (or
 * arbitrary frequency) control loops with minimal jitter.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__HIGH_RESOLUTION_TIMER_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__HIGH_RESOLUTION_TIMER_HPP_

#include <chrono>
#include <cstdint>

namespace rt_controller_framework {
namespace realtime_utils {

/**
 * @brief A high-resolution periodic timer for real-time control loops.
 *
 * Usage:
 * @code
 *   HighResolutionTimer timer(1000);  // 1kHz
 *   timer.start();
 *   while (running) {
 *     timer.wait_for_next_cycle();
 *     // ... do RT work ...
 *     auto dt = timer.get_actual_period_us();
 *   }
 * @endcode
 */
class HighResolutionTimer {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = Clock::duration;

  /**
   * @brief Construct a timer with the given frequency.
   * @param frequency_hz Desired loop frequency in Hz (e.g. 1000 for 1kHz).
   */
  explicit HighResolutionTimer(uint32_t frequency_hz);

  /**
   * @brief Reset and start the timer. Records the initial time point.
   */
  void start();

  /**
   * @brief Block until the next cycle boundary.
   *
   * Uses std::this_thread::sleep_until() for precise, non-spinning waits.
   * Automatically advances the target wakeup time by one period.
   */
  void wait_for_next_cycle();

  /**
   * @brief Get the configured period in microseconds.
   * @return Period in µs.
   */
  [[nodiscard]] int64_t get_period_us() const;

  /**
   * @brief Get the actual elapsed time of the last completed cycle.
   * @return Actual period in µs.
   */
  [[nodiscard]] int64_t get_actual_period_us() const;

  /**
   * @brief Get the configured frequency.
   * @return Frequency in Hz.
   */
  [[nodiscard]] uint32_t get_frequency_hz() const;

  /**
   * @brief Get the number of completed cycles since start().
   * @return Cycle count.
   */
  [[nodiscard]] uint64_t get_cycle_count() const;

  /**
   * @brief Get the total elapsed time since start().
   * @return Elapsed time in microseconds.
   */
  [[nodiscard]] int64_t get_elapsed_us() const;

  /**
   * @brief Check if a deadline overrun occurred on the last cycle.
   * @return true if the actual period exceeded 1.5× the configured period.
   */
  [[nodiscard]] bool has_overrun() const;

 private:
  uint32_t frequency_hz_;
  Duration period_;
  TimePoint start_time_;
  TimePoint next_wakeup_;
  TimePoint last_wakeup_;
  int64_t actual_period_us_{0};
  uint64_t cycle_count_{0};
  bool overrun_{false};
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__HIGH_RESOLUTION_TIMER_HPP_
