/**
 * @file test_high_resolution_timer.cpp
 * @brief GTest unit tests for the high-resolution periodic timer.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "rt_controller_framework/realtime_utils/high_resolution_timer.hpp"

using rt_controller_framework::realtime_utils::HighResolutionTimer;

TEST(HighResolutionTimerTest, Construction) {
  HighResolutionTimer timer(1000);
  EXPECT_EQ(timer.get_frequency_hz(), 1000u);
  EXPECT_EQ(timer.get_period_us(), 1000);
}

TEST(HighResolutionTimerTest, DifferentFrequencies) {
  HighResolutionTimer t500(500);
  EXPECT_EQ(t500.get_frequency_hz(), 500u);
  EXPECT_EQ(t500.get_period_us(), 2000);

  HighResolutionTimer t100(100);
  EXPECT_EQ(t100.get_frequency_hz(), 100u);
  EXPECT_EQ(t100.get_period_us(), 10000);
}

TEST(HighResolutionTimerTest, CycleCount) {
  HighResolutionTimer timer(1000);
  timer.start();
  EXPECT_EQ(timer.get_cycle_count(), 0u);

  // Run a few cycles
  for (int i = 0; i < 5; ++i) {
    timer.wait_for_next_cycle();
  }
  EXPECT_EQ(timer.get_cycle_count(), 5u);
}

TEST(HighResolutionTimerTest, PeriodAccuracy) {
  // Use a lower frequency (100 Hz = 10ms period) to allow for OS scheduling
  constexpr uint32_t kFreq = 100;
  constexpr int kCycles = 10;
  constexpr int64_t kExpectedPeriodUs = 10000;
  constexpr int64_t kToleranceUs = 2000;  // 2ms tolerance for CI/non-RT

  HighResolutionTimer timer(kFreq);
  timer.start();

  for (int i = 0; i < kCycles; ++i) {
    timer.wait_for_next_cycle();
    int64_t actual = timer.get_actual_period_us();

    EXPECT_NEAR(actual, kExpectedPeriodUs, kToleranceUs)
        << "Cycle " << i << " period: " << actual << " µs";
  }
}

TEST(HighResolutionTimerTest, ElapsedTime) {
  HighResolutionTimer timer(100);  // 100 Hz = 10ms period
  timer.start();

  // Run 5 cycles (should take ~50 ms)
  for (int i = 0; i < 5; ++i) {
    timer.wait_for_next_cycle();
  }

  int64_t elapsed = timer.get_elapsed_us();
  // Should be approximately 50,000 µs ± 10ms tolerance
  EXPECT_GT(elapsed, 40000);
  EXPECT_LT(elapsed, 70000);
}

TEST(HighResolutionTimerTest, StartResetsState) {
  HighResolutionTimer timer(100);

  timer.start();
  timer.wait_for_next_cycle();
  EXPECT_EQ(timer.get_cycle_count(), 1u);

  // Restart should reset
  timer.start();
  EXPECT_EQ(timer.get_cycle_count(), 0u);
}
