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

#include <memory>

#include "rt_controller_framework/controller/rt_controller.hpp"
#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"
#include "rt_controller_framework/realtime_utils/lock_free_queue.hpp"

using rt_controller_framework::controller::RTCommand;
using rt_controller_framework::controller::RTController;
using rt_controller_framework::realtime_utils::JitterMonitor;
using rt_controller_framework::realtime_utils::LockFreeQueue;

// --- RTCommand struct tests ---

TEST(RTControllerTest, RTCommandDefaults)
{
  RTCommand cmd;
  EXPECT_EQ(cmd.joint_index, 0u);
  EXPECT_DOUBLE_EQ(cmd.target_position, 0.0);
  EXPECT_DOUBLE_EQ(cmd.target_velocity, 0.0);
}

TEST(RTControllerTest, RTCommandQueue)
{
  LockFreeQueue<RTCommand, 16> queue;

  RTCommand cmd1{0, 1.5, 0.1};
  RTCommand cmd2{1, -0.5, 0.0};

  EXPECT_TRUE(queue.try_push(cmd1));
  EXPECT_TRUE(queue.try_push(cmd2));

  auto pop1 = queue.try_pop();
  ASSERT_TRUE(pop1.has_value());
  EXPECT_EQ(pop1->joint_index, 0u);
  EXPECT_DOUBLE_EQ(pop1->target_position, 1.5);

  auto pop2 = queue.try_pop();
  ASSERT_TRUE(pop2.has_value());
  EXPECT_EQ(pop2->joint_index, 1u);
  EXPECT_DOUBLE_EQ(pop2->target_position, -0.5);
}

// --- Controller plugin tests ---

TEST(RTControllerTest, ControllerConstruction)
{
  // Verifies that the controller can be constructed without crashing
  auto controller = std::make_shared<RTController>();
  EXPECT_NE(controller, nullptr);
}

// --- Integration with JitterMonitor ---

TEST(RTControllerTest, JitterMonitorIntegration)
{
  JitterMonitor<100> monitor(1000);

  // Simulate some update cycles
  for (int i = 0; i < 50; ++i) {
    // Simulate ~1000 us period with some jitter
    int64_t period = 1000 + (i % 5) - 2;
    monitor.record_cycle(period);
  }

  auto stats = monitor.get_statistics();
  EXPECT_EQ(stats.total_cycles, 50u);
  EXPECT_LT(stats.max_jitter_us, 5.0);  // Max jitter should be small
  EXPECT_NEAR(stats.mean_period_us, 1000.0, 5.0);
}

// --- PD Control Logic (isolated) ---

TEST(RTControllerTest, PDControlComputation)
{
  // Test the PD control math in isolation
  const double kp = 100.0;
  const double kd = 10.0;

  const double target_pos = 1.0;
  const double current_pos = 0.0;
  const double target_vel = 0.0;
  const double current_vel = 0.0;

  double pos_error = target_pos - current_pos;
  double vel_error = target_vel - current_vel;
  double effort = kp * pos_error + kd * vel_error;

  EXPECT_DOUBLE_EQ(effort, 100.0);  // kp * 1.0 + kd * 0.0

  // With velocity
  double current_vel2 = 0.5;
  double vel_error2 = target_vel - current_vel2;
  double effort2 = kp * pos_error + kd * vel_error2;

  EXPECT_DOUBLE_EQ(effort2, 95.0);  // 100 * 1.0 + 10 * (-0.5)
}

TEST(RTControllerTest, PDControlConvergence)
{
  // Simulate PD control convergence over time
  const double kp = 100.0;
  const double kd = 20.0;
  const double dt = 0.001;  // 1kHz
  const double inertia = 1.0;
  const double target = 1.0;

  double pos = 0.0;
  double vel = 0.0;

  for (int i = 0; i < 5000; ++i) {
    double pos_error = target - pos;
    double effort = kp * pos_error + kd * (0.0 - vel);

    double accel = effort / inertia;
    vel += accel * dt;
    pos += vel * dt;
  }

  // After 5 seconds at 1kHz, should converge to target
  EXPECT_NEAR(pos, target, 0.01);
  EXPECT_NEAR(vel, 0.0, 0.1);
}
