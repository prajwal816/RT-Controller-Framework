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

/**
 * @file rt_loop_example.cpp
 * @brief Minimal example of a 1kHz real-time loop with jitter measurement.
 *
 * Demonstrates the use of HighResolutionTimer and JitterMonitor from the
 * rt_controller_framework::realtime_utils namespace. This is a standalone
 * program that does not require a running ROS2 system.
 *
 * Build:
 *   colcon build --packages-select rt_controller_framework
 *
 * Run:
 *   ros2 run rt_controller_framework rt_loop_example
 */

#include <cinttypes>
#include <cstdio>
#include <cstdlib>

#include "rt_controller_framework/realtime_utils/high_resolution_timer.hpp"
#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"
#include "rt_controller_framework/realtime_utils/lock_free_queue.hpp"
#include "rt_controller_framework/realtime_utils/memory_pool.hpp"

using rt_controller_framework::realtime_utils::HighResolutionTimer;
using rt_controller_framework::realtime_utils::JitterMonitor;
using rt_controller_framework::realtime_utils::LockFreeQueue;
using rt_controller_framework::realtime_utils::MemoryPool;

/**
 * @brief Simple data structure used in the memory pool demo.
 */
struct ControlData
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

int main(int /*argc*/, char ** /*argv*/)
{
  constexpr uint32_t kFrequencyHz = 1000;      // 1kHz
  constexpr uint64_t kNumCycles = 5000;         // 5 seconds
  constexpr std::size_t kQueueCapacity = 32;
  constexpr std::size_t kPoolSize = 16;

  std::printf("=============================================================\n");
  std::printf("       RT Controller Framework - Loop Example\n");
  std::printf("=============================================================\n");
  std::printf("  Frequency:  %5u Hz\n", kFrequencyHz);
  std::printf(
    "  Cycles:     %5" PRIu64 "\n",
    kNumCycles);
  std::printf(
    "  Duration:   %5.1f s\n",
    static_cast<double>(kNumCycles) / kFrequencyHz);
  std::printf("=============================================================\n\n");

  // ─── Initialize RT primitives ─────────────────────────────────────────
  HighResolutionTimer timer(kFrequencyHz);
  JitterMonitor<1000> jitter_monitor(kFrequencyHz);
  LockFreeQueue<double, kQueueCapacity> command_queue;
  MemoryPool<ControlData, kPoolSize> data_pool;

  // ─── Demonstrate lock-free queue ──────────────────────────────────────
  std::printf("[Demo] Lock-Free Queue (capacity=%zu):\n", command_queue.capacity());
  for (int i = 0; i < 5; ++i) {
    bool ok = command_queue.try_push(static_cast<double>(i) * 0.1);
    std::printf("  push(%.1f) -> %s\n", i * 0.1, ok ? "OK" : "FULL");
  }
  while (auto val = command_queue.try_pop()) {
    std::printf("  pop() -> %.1f\n", *val);
  }
  std::printf("\n");

  // ─── Demonstrate memory pool ──────────────────────────────────────────
  std::printf("[Demo] Memory Pool (capacity=%zu):\n", data_pool.capacity());
  auto * d1 = data_pool.allocate();
  auto * d2 = data_pool.allocate();
  if (d1 && d2) {
    d1->position = 1.0;
    d2->velocity = 2.0;
    std::printf("  Allocated 2 blocks (available: %zu)\n", data_pool.available());
    data_pool.deallocate(d1);
    data_pool.deallocate(d2);
    std::printf("  Freed 2 blocks (available: %zu)\n", data_pool.available());
  }
  std::printf("\n");

  // ─── Run the 1kHz loop ────────────────────────────────────────────────
  std::printf(
    "[RT Loop] Starting %" PRIu64 " cycles at %u Hz...\n\n",
    kNumCycles, kFrequencyHz);

  timer.start();
  for (uint64_t cycle = 0; cycle < kNumCycles; ++cycle) {
    timer.wait_for_next_cycle();

    // Record jitter
    jitter_monitor.record_cycle(timer.get_actual_period_us());

    // Simulated workload: trivial computation (avoids optimization removal)
    volatile double dummy = 0.0;
    for (int i = 0; i < 10; ++i) {
      dummy += static_cast<double>(i) * 0.001;
    }
    (void)dummy;

    // Periodic progress report (every 1000 cycles = 1 second)
    if ((cycle + 1) % 1000 == 0) {
      auto stats = jitter_monitor.get_statistics();
      std::printf(
        "  [%5" PRIu64 "s] Cycles: %" PRIu64 " | Mean Jitter: %.2f us | "
        "Max Jitter: %.2f us | Mean Period: %.2f us\n",
        (cycle + 1) / kFrequencyHz,
        stats.total_cycles,
        stats.mean_jitter_us,
        stats.max_jitter_us,
        stats.mean_period_us);
    }
  }

  // ─── Final Report ─────────────────────────────────────────────────────
  std::printf("\n%s", jitter_monitor.format_summary().c_str());

  auto stats = jitter_monitor.get_statistics();
  bool pass = stats.max_jitter_us < 50.0;

  std::printf("\n=============================================================\n");
  std::printf(
    "  RESULT: %s\n",
    pass ? "PASS" : "FAIL");
  std::printf(
    "  Max Jitter: %8.2f us (threshold: 50.00 us)\n",
    stats.max_jitter_us);
  std::printf("=============================================================\n");

  return pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
