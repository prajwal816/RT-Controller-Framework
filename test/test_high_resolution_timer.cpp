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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "rt_controller_framework/realtime_utils/high_resolution_timer.hpp"

using rt_controller_framework::realtime_utils::HighResolutionTimer;

TEST(HighResolutionTimerTest, Construction)
{
  HighResolutionTimer timer(1000);
  EXPECT_EQ(timer.get_frequency_hz(), 1000u);
}

TEST(HighResolutionTimerTest, DifferentFrequencies)
{
  HighResolutionTimer t500(500);
  EXPECT_EQ(t500.get_frequency_hz(), 500u);

  HighResolutionTimer t100(100);
  EXPECT_EQ(t100.get_frequency_hz(), 100u);
}

TEST(HighResolutionTimerTest, CycleCount)
{
  HighResolutionTimer timer(1000);
  timer.start();
  EXPECT_EQ(timer.get_cycle_count(), 0u);

  // Run a few cycles
  for (int i = 0; i < 5; ++i) {
    timer.wait_for_next_cycle();
  }
  EXPECT_EQ(timer.get_cycle_count(), 5u);
}

TEST(HighResolutionTimerTest, PeriodAccuracy)
{
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
      << "Cycle " << i << " period: " << actual << " us";
  }
}

TEST(HighResolutionTimerTest, ElapsedTime)
{
  HighResolutionTimer timer(100);  // 100 Hz = 10ms period
  timer.start();

  // Run 5 cycles (should take ~50 ms)
  for (int i = 0; i < 5; ++i) {
    timer.wait_for_next_cycle();
  }

  auto elapsed = timer.get_elapsed();
  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
    elapsed).count();
  // Should be approximately 50,000 us +/- 10ms tolerance
  EXPECT_GT(elapsed_us, 40000);
  EXPECT_LT(elapsed_us, 70000);
}

TEST(HighResolutionTimerTest, StartResetsState)
{
  HighResolutionTimer timer(100);

  timer.start();
  timer.wait_for_next_cycle();
  EXPECT_EQ(timer.get_cycle_count(), 1u);

  // Restart should reset
  timer.start();
  EXPECT_EQ(timer.get_cycle_count(), 0u);
}
