/**
 * @file test_jitter_monitor.cpp
 * @brief GTest unit tests for the jitter measurement module.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"

using rt_controller_framework::realtime_utils::JitterMonitor;

// ─── Basic Statistics ───────────────────────────────────────────────────────

TEST(JitterMonitorTest, InitialState) {
  JitterMonitor<100> monitor(1000);  // 1kHz
  auto stats = monitor.get_statistics();

  EXPECT_EQ(stats.total_cycles, 0u);
  EXPECT_EQ(stats.overrun_count, 0u);
  EXPECT_DOUBLE_EQ(stats.mean_jitter_us, 0.0);
  EXPECT_DOUBLE_EQ(stats.max_jitter_us, 0.0);
  EXPECT_EQ(stats.expected_period_us, 1000);
}

TEST(JitterMonitorTest, PerfectTiming) {
  JitterMonitor<100> monitor(1000);

  // Record cycles with exactly 1000 µs period
  for (int i = 0; i < 100; ++i) {
    monitor.record_cycle(1000);
  }

  auto stats = monitor.get_statistics();
  EXPECT_EQ(stats.total_cycles, 100u);
  EXPECT_DOUBLE_EQ(stats.mean_jitter_us, 0.0);
  EXPECT_DOUBLE_EQ(stats.max_jitter_us, 0.0);
  EXPECT_DOUBLE_EQ(stats.mean_period_us, 1000.0);
  EXPECT_EQ(stats.overrun_count, 0u);
}

TEST(JitterMonitorTest, KnownJitter) {
  JitterMonitor<100> monitor(1000);

  // Record cycles alternating between 990 and 1010 µs
  for (int i = 0; i < 50; ++i) {
    monitor.record_cycle(990);   // jitter = 10 µs
    monitor.record_cycle(1010);  // jitter = 10 µs
  }

  auto stats = monitor.get_statistics();
  EXPECT_EQ(stats.total_cycles, 100u);
  EXPECT_DOUBLE_EQ(stats.mean_jitter_us, 10.0);
  EXPECT_DOUBLE_EQ(stats.max_jitter_us, 10.0);
  EXPECT_NEAR(stats.mean_period_us, 1000.0, 0.01);
  EXPECT_EQ(stats.overrun_count, 0u);
}

TEST(JitterMonitorTest, MaxJitter) {
  JitterMonitor<100> monitor(1000);

  monitor.record_cycle(1000);
  monitor.record_cycle(1000);
  monitor.record_cycle(1050);  // 50 µs jitter
  monitor.record_cycle(1000);

  auto stats = monitor.get_statistics();
  EXPECT_DOUBLE_EQ(stats.max_jitter_us, 50.0);
}

// ─── Overrun Detection ──────────────────────────────────────────────────────

TEST(JitterMonitorTest, OverrunDetection) {
  JitterMonitor<100> monitor(1000);

  // Normal cycles
  for (int i = 0; i < 10; ++i) {
    monitor.record_cycle(1000);
  }

  // Overrun cycle (> 2× expected period = 2000 µs)
  monitor.record_cycle(2100);
  monitor.record_cycle(2500);

  auto stats = monitor.get_statistics();
  EXPECT_EQ(stats.overrun_count, 2u);
}

// ─── Ring Buffer ────────────────────────────────────────────────────────────

TEST(JitterMonitorTest, BufferSampleCount) {
  JitterMonitor<10> monitor(1000);

  // Fill partially
  for (int i = 0; i < 5; ++i) {
    monitor.record_cycle(1000 + i);
  }
  EXPECT_EQ(monitor.buffer_sample_count(), 5u);

  // Fill to capacity
  for (int i = 5; i < 10; ++i) {
    monitor.record_cycle(1000 + i);
  }
  EXPECT_EQ(monitor.buffer_sample_count(), 10u);

  // Overfill — should wrap
  for (int i = 10; i < 15; ++i) {
    monitor.record_cycle(1000 + i);
  }
  EXPECT_EQ(monitor.buffer_sample_count(), 10u);  // Capped at buffer size
}

TEST(JitterMonitorTest, GetJitterSample) {
  JitterMonitor<10> monitor(1000);

  // Push 5 samples with known jitter
  monitor.record_cycle(1010);  // jitter = 10
  monitor.record_cycle(1020);  // jitter = 20
  monitor.record_cycle(980);   // jitter = 20
  monitor.record_cycle(1000);  // jitter = 0

  EXPECT_DOUBLE_EQ(monitor.get_jitter_sample(0), 10.0);
  EXPECT_DOUBLE_EQ(monitor.get_jitter_sample(1), 20.0);
  EXPECT_DOUBLE_EQ(monitor.get_jitter_sample(2), 20.0);
  EXPECT_DOUBLE_EQ(monitor.get_jitter_sample(3), 0.0);

  // Out of range
  EXPECT_DOUBLE_EQ(monitor.get_jitter_sample(10), 0.0);
}

// ─── Reset ──────────────────────────────────────────────────────────────────

TEST(JitterMonitorTest, Reset) {
  JitterMonitor<100> monitor(1000);

  for (int i = 0; i < 50; ++i) {
    monitor.record_cycle(1010);
  }
  EXPECT_EQ(monitor.get_statistics().total_cycles, 50u);

  monitor.reset();

  auto stats = monitor.get_statistics();
  EXPECT_EQ(stats.total_cycles, 0u);
  EXPECT_DOUBLE_EQ(stats.mean_jitter_us, 0.0);
  EXPECT_DOUBLE_EQ(stats.max_jitter_us, 0.0);
  EXPECT_EQ(monitor.buffer_sample_count(), 0u);
}

// ─── Format Summary ─────────────────────────────────────────────────────────

TEST(JitterMonitorTest, FormatSummary) {
  JitterMonitor<100> monitor(1000);

  monitor.record_cycle(1000);
  monitor.record_cycle(1010);

  std::string summary = monitor.format_summary();
  EXPECT_FALSE(summary.empty());
  EXPECT_NE(summary.find("1000 Hz"), std::string::npos);
  EXPECT_NE(summary.find("Cycles"), std::string::npos);
}

// ─── Standard Deviation ─────────────────────────────────────────────────────

TEST(JitterMonitorTest, StddevComputation) {
  JitterMonitor<100> monitor(1000);

  // All same jitter → stddev should be 0
  for (int i = 0; i < 100; ++i) {
    monitor.record_cycle(1010);  // constant 10 µs jitter
  }

  auto stats = monitor.get_statistics();
  EXPECT_NEAR(stats.stddev_jitter_us, 0.0, 0.01);
}
