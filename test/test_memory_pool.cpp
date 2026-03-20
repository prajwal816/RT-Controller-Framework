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

#include <set>
#include <vector>

#include "rt_controller_framework/realtime_utils/memory_pool.hpp"

using rt_controller_framework::realtime_utils::MemoryPool;

struct TestData
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  int id{0};
};

// --- Basic Operations ---

TEST(MemoryPoolTest, InitialState)
{
  MemoryPool<TestData, 8> pool;
  EXPECT_EQ(pool.available(), 8u);
  EXPECT_EQ(pool.capacity(), 8u);
}

TEST(MemoryPoolTest, AllocateAndDeallocate)
{
  MemoryPool<TestData, 4> pool;

  auto * ptr = pool.allocate();
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(pool.available(), 3u);

  // Use the allocated memory
  ptr->x = 1.0;
  ptr->y = 2.0;
  ptr->z = 3.0;
  ptr->id = 42;
  EXPECT_DOUBLE_EQ(ptr->x, 1.0);
  EXPECT_EQ(ptr->id, 42);

  pool.deallocate(ptr);
  EXPECT_EQ(pool.available(), 4u);
}

TEST(MemoryPoolTest, AllocateMultiple)
{
  MemoryPool<TestData, 4> pool;

  std::vector<TestData *> ptrs;
  for (int i = 0; i < 4; ++i) {
    auto * p = pool.allocate();
    ASSERT_NE(p, nullptr);
    p->id = i;
    ptrs.push_back(p);
  }

  EXPECT_EQ(pool.available(), 0u);

  // All pointers should be unique
  std::set<TestData *> unique_ptrs(ptrs.begin(), ptrs.end());
  EXPECT_EQ(unique_ptrs.size(), 4u);

  // Verify values
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(ptrs[i]->id, i);
  }

  // Free all
  for (auto * p : ptrs) {
    pool.deallocate(p);
  }
  EXPECT_EQ(pool.available(), 4u);
}

// --- Boundary Conditions ---

TEST(MemoryPoolTest, PoolExhaustion)
{
  MemoryPool<TestData, 2> pool;

  auto * p1 = pool.allocate();
  auto * p2 = pool.allocate();
  ASSERT_NE(p1, nullptr);
  ASSERT_NE(p2, nullptr);

  // Pool is now exhausted
  auto * p3 = pool.allocate();
  EXPECT_EQ(p3, nullptr);

  // Free one and allocate again
  pool.deallocate(p1);
  auto * p4 = pool.allocate();
  ASSERT_NE(p4, nullptr);
  EXPECT_EQ(pool.available(), 0u);

  pool.deallocate(p2);
  pool.deallocate(p4);
}

TEST(MemoryPoolTest, DeallocateNullptr)
{
  MemoryPool<TestData, 4> pool;

  // Should not crash or change state
  pool.deallocate(nullptr);
  EXPECT_EQ(pool.available(), 4u);
}

TEST(MemoryPoolTest, DeallocateForeignPointer)
{
  MemoryPool<TestData, 4> pool;
  TestData foreign;

  // Should silently ignore foreign pointer
  pool.deallocate(&foreign);
  EXPECT_EQ(pool.available(), 4u);
}

// --- Reset ---

TEST(MemoryPoolTest, Reset)
{
  MemoryPool<TestData, 4> pool;

  pool.allocate();
  pool.allocate();
  EXPECT_EQ(pool.available(), 2u);

  pool.reset();
  EXPECT_EQ(pool.available(), 4u);

  // Should be able to allocate all blocks again
  for (int i = 0; i < 4; ++i) {
    ASSERT_NE(pool.allocate(), nullptr);
  }
  EXPECT_EQ(pool.available(), 0u);
}

// --- Alternating allocate/free pattern ---

TEST(MemoryPoolTest, AlternatingAllocFree)
{
  MemoryPool<TestData, 4> pool;

  for (int round = 0; round < 100; ++round) {
    auto * p = pool.allocate();
    ASSERT_NE(p, nullptr) << "Failed at round " << round;
    p->id = round;
    EXPECT_EQ(p->id, round);
    pool.deallocate(p);
  }

  EXPECT_EQ(pool.available(), 4u);
}

// --- Primitive Type ---

TEST(MemoryPoolTest, PrimitiveType)
{
  MemoryPool<double, 8> pool;

  auto * p = pool.allocate();
  ASSERT_NE(p, nullptr);
  *p = 3.14159;
  EXPECT_DOUBLE_EQ(*p, 3.14159);

  pool.deallocate(p);
  EXPECT_EQ(pool.available(), 8u);
}
