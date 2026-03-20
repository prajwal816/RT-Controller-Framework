/**
 * @file jitter_monitor.hpp
 * @brief Real-time jitter measurement and statistics module.
 *
 * Records per-cycle timing deviations from the ideal period,
 * computes running statistics (mean, max, standard deviation),
 * and maintains a ring buffer of recent samples for analysis.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__JITTER_MONITOR_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__JITTER_MONITOR_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace rt_controller_framework {
namespace realtime_utils {

/**
 * @brief Jitter measurement and statistics collector.
 *
 * @tparam BufferSize Number of recent jitter samples to retain in the
 *                    ring buffer. Default: 1000 (1 second at 1kHz).
 *
 * Usage:
 * @code
 *   JitterMonitor<1000> monitor(1000);  // 1kHz expected
 *   while (running) {
 *     // ... do RT work ...
 *     monitor.record_cycle(actual_period_us);
 *     auto stats = monitor.get_statistics();
 *   }
 * @endcode
 */
template <std::size_t BufferSize = 1000>
class JitterMonitor {
 public:
  /**
   * @brief Statistics snapshot.
   */
  struct Statistics {
    double mean_jitter_us{0.0};     ///< Mean absolute jitter in µs
    double max_jitter_us{0.0};      ///< Maximum observed jitter in µs
    double stddev_jitter_us{0.0};   ///< Standard deviation of jitter in µs
    double mean_period_us{0.0};     ///< Mean actual period in µs
    int64_t expected_period_us{0};  ///< Expected (ideal) period in µs
    uint64_t total_cycles{0};       ///< Total cycles recorded
    uint64_t overrun_count{0};      ///< Cycles exceeding 2× expected period
  };

  /**
   * @brief Construct a jitter monitor.
   * @param expected_frequency_hz The expected loop frequency (e.g. 1000).
   */
  explicit JitterMonitor(uint32_t expected_frequency_hz)
      : expected_period_us_(1000000 / expected_frequency_hz),
        expected_frequency_hz_(expected_frequency_hz) {}

  /**
   * @brief Record one cycle's actual period.
   * @param actual_period_us The measured period of the last cycle in µs.
   */
  void record_cycle(int64_t actual_period_us) noexcept {
    const double jitter = static_cast<double>(actual_period_us - expected_period_us_);
    const double abs_jitter = jitter > 0 ? jitter : -jitter;

    // Update ring buffer
    jitter_buffer_[write_index_] = abs_jitter;
    period_buffer_[write_index_] = static_cast<double>(actual_period_us);
    write_index_ = (write_index_ + 1) % BufferSize;
    if (sample_count_ < BufferSize) {
      ++sample_count_;
    }

    // Update running statistics
    ++total_cycles_;
    sum_jitter_ += abs_jitter;
    sum_jitter_sq_ += abs_jitter * abs_jitter;
    sum_period_ += static_cast<double>(actual_period_us);

    if (abs_jitter > max_jitter_) {
      max_jitter_ = abs_jitter;
    }

    if (actual_period_us > 2 * expected_period_us_) {
      ++overrun_count_;
    }
  }

  /**
   * @brief Get a snapshot of the current statistics.
   * @return Statistics struct with computed values.
   */
  [[nodiscard]] Statistics get_statistics() const noexcept {
    Statistics stats;
    stats.expected_period_us = expected_period_us_;
    stats.total_cycles = total_cycles_;
    stats.overrun_count = overrun_count_;
    stats.max_jitter_us = max_jitter_;

    if (total_cycles_ > 0) {
      stats.mean_jitter_us = sum_jitter_ / static_cast<double>(total_cycles_);
      stats.mean_period_us = sum_period_ / static_cast<double>(total_cycles_);

      if (total_cycles_ > 1) {
        const double variance =
            (sum_jitter_sq_ -
             (sum_jitter_ * sum_jitter_) / static_cast<double>(total_cycles_)) /
            static_cast<double>(total_cycles_ - 1);
        stats.stddev_jitter_us = (variance > 0.0) ? std::sqrt(variance) : 0.0;
      }
    }

    return stats;
  }

  /**
   * @brief Reset all statistics and the ring buffer.
   */
  void reset() noexcept {
    write_index_ = 0;
    sample_count_ = 0;
    total_cycles_ = 0;
    overrun_count_ = 0;
    sum_jitter_ = 0.0;
    sum_jitter_sq_ = 0.0;
    sum_period_ = 0.0;
    max_jitter_ = 0.0;
    jitter_buffer_.fill(0.0);
    period_buffer_.fill(0.0);
  }

  /**
   * @brief Get the number of samples currently in the ring buffer.
   */
  [[nodiscard]] std::size_t buffer_sample_count() const noexcept {
    return sample_count_;
  }

  /**
   * @brief Get a specific jitter sample from the ring buffer.
   * @param index Index into the buffer (0 = oldest retained sample).
   * @return Jitter value in µs, or 0.0 if index is out of range.
   */
  [[nodiscard]] double get_jitter_sample(std::size_t index) const noexcept {
    if (index >= sample_count_) return 0.0;

    std::size_t actual_index;
    if (sample_count_ < BufferSize) {
      actual_index = index;
    } else {
      actual_index = (write_index_ + index) % BufferSize;
    }
    return jitter_buffer_[actual_index];
  }

  /**
   * @brief Format a summary string of the current statistics.
   * @return Human-readable statistics summary.
   */
  [[nodiscard]] std::string format_summary() const {
    auto stats = get_statistics();
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "RT Jitter Report [%u Hz]\n"
        "  Cycles:       %lu\n"
        "  Mean Jitter:  %.2f us\n"
        "  Max Jitter:   %.2f us\n"
        "  Stddev:       %.2f us\n"
        "  Mean Period:  %.2f us (expected: %ld us)\n"
        "  Overruns:     %lu\n",
        expected_frequency_hz_,
        static_cast<unsigned long>(stats.total_cycles),
        stats.mean_jitter_us,
        stats.max_jitter_us,
        stats.stddev_jitter_us,
        stats.mean_period_us,
        static_cast<long>(stats.expected_period_us),
        static_cast<unsigned long>(stats.overrun_count));
    return std::string(buf);
  }

 private:
  int64_t expected_period_us_;
  uint32_t expected_frequency_hz_;

  // Ring buffer
  std::array<double, BufferSize> jitter_buffer_{};
  std::array<double, BufferSize> period_buffer_{};
  std::size_t write_index_{0};
  std::size_t sample_count_{0};

  // Running stats
  uint64_t total_cycles_{0};
  uint64_t overrun_count_{0};
  double sum_jitter_{0.0};
  double sum_jitter_sq_{0.0};
  double sum_period_{0.0};
  double max_jitter_{0.0};
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

// Include cmath for std::sqrt used in the template
#include <cmath>

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__JITTER_MONITOR_HPP_
