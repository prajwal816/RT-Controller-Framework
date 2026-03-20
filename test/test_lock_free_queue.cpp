/**
 * @file test_lock_free_queue.cpp
 * @brief GTest unit tests for the SPSC lock-free ring buffer.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "rt_controller_framework/realtime_utils/lock_free_queue.hpp"

using rt_controller_framework::realtime_utils::LockFreeQueue;

// ─── Basic Operations ───────────────────────────────────────────────────────

TEST(LockFreeQueueTest, EmptyOnConstruction) {
  LockFreeQueue<int, 8> q;
  EXPECT_TRUE(q.empty());
  EXPECT_EQ(q.size(), 0u);
  EXPECT_EQ(q.capacity(), 8u);
}

TEST(LockFreeQueueTest, PushAndPop) {
  LockFreeQueue<int, 4> q;

  EXPECT_TRUE(q.try_push(42));
  EXPECT_FALSE(q.empty());
  EXPECT_EQ(q.size(), 1u);

  auto val = q.try_pop();
  ASSERT_TRUE(val.has_value());
  EXPECT_EQ(*val, 42);
  EXPECT_TRUE(q.empty());
}

TEST(LockFreeQueueTest, FIFO_Order) {
  LockFreeQueue<int, 8> q;

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(q.try_push(i * 10));
  }
  EXPECT_EQ(q.size(), 5u);

  for (int i = 0; i < 5; ++i) {
    auto val = q.try_pop();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, i * 10);
  }
  EXPECT_TRUE(q.empty());
}

// ─── Boundary Conditions ────────────────────────────────────────────────────

TEST(LockFreeQueueTest, FullQueue) {
  LockFreeQueue<int, 4> q;

  // Fill to capacity
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(q.try_push(i));
  }
  EXPECT_EQ(q.size(), 4u);

  // Push should fail when full
  EXPECT_FALSE(q.try_push(99));
}

TEST(LockFreeQueueTest, EmptyQueuePop) {
  LockFreeQueue<int, 4> q;
  auto val = q.try_pop();
  EXPECT_FALSE(val.has_value());
}

TEST(LockFreeQueueTest, WrapAround) {
  LockFreeQueue<int, 4> q;

  // Fill and drain twice to exercise wrap-around
  for (int round = 0; round < 3; ++round) {
    for (int i = 0; i < 4; ++i) {
      EXPECT_TRUE(q.try_push(round * 100 + i));
    }

    for (int i = 0; i < 4; ++i) {
      auto val = q.try_pop();
      ASSERT_TRUE(val.has_value());
      EXPECT_EQ(*val, round * 100 + i);
    }
    EXPECT_TRUE(q.empty());
  }
}

TEST(LockFreeQueueTest, Reset) {
  LockFreeQueue<int, 8> q;

  q.try_push(1);
  q.try_push(2);
  EXPECT_EQ(q.size(), 2u);

  q.reset();
  EXPECT_TRUE(q.empty());
  EXPECT_EQ(q.size(), 0u);

  // Should be able to push again after reset
  EXPECT_TRUE(q.try_push(3));
  auto val = q.try_pop();
  ASSERT_TRUE(val.has_value());
  EXPECT_EQ(*val, 3);
}

// ─── Concurrent SPSC Test ───────────────────────────────────────────────────

TEST(LockFreeQueueTest, ConcurrentSPSC) {
  constexpr int kNumElements = 10000;
  LockFreeQueue<int, 256> q;

  std::atomic<bool> producer_done{false};
  std::vector<int> received;
  received.reserve(kNumElements);

  // Producer thread
  std::thread producer([&]() {
    for (int i = 0; i < kNumElements; ++i) {
      while (!q.try_push(i)) {
        // Spin until space available
        std::this_thread::yield();
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  // Consumer thread
  std::thread consumer([&]() {
    while (true) {
      auto val = q.try_pop();
      if (val.has_value()) {
        received.push_back(*val);
        if (received.size() == kNumElements) break;
      } else if (producer_done.load(std::memory_order_acquire) && q.empty()) {
        break;
      } else {
        std::this_thread::yield();
      }
    }
  });

  producer.join();
  consumer.join();

  // Verify all elements received in order
  ASSERT_EQ(received.size(), static_cast<size_t>(kNumElements));
  for (int i = 0; i < kNumElements; ++i) {
    EXPECT_EQ(received[i], i) << "Mismatch at index " << i;
  }
}

// ─── Struct Type ────────────────────────────────────────────────────────────

TEST(LockFreeQueueTest, StructType) {
  struct Cmd {
    int id;
    double value;
  };

  LockFreeQueue<Cmd, 4> q;

  EXPECT_TRUE(q.try_push({1, 3.14}));
  EXPECT_TRUE(q.try_push({2, 2.71}));

  auto v1 = q.try_pop();
  ASSERT_TRUE(v1.has_value());
  EXPECT_EQ(v1->id, 1);
  EXPECT_DOUBLE_EQ(v1->value, 3.14);

  auto v2 = q.try_pop();
  ASSERT_TRUE(v2.has_value());
  EXPECT_EQ(v2->id, 2);
  EXPECT_DOUBLE_EQ(v2->value, 2.71);
}
