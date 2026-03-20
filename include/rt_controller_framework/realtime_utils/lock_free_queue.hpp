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

/// @file lock_free_queue.hpp
/// @brief Wait-free Single-Producer Single-Consumer (SPSC) lock-free queue.

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__LOCK_FREE_QUEUE_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__LOCK_FREE_QUEUE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace rt_controller_framework
{
namespace realtime_utils
{

/// @brief Lock-free SPSC ring buffer for real-time inter-thread communication.
/// @tparam T Element type (must be trivially copyable for best performance).
/// @tparam Capacity Maximum number of elements (compile-time constant).
template<typename T, std::size_t Capacity>
class LockFreeQueue
{
public:
  LockFreeQueue() = default;

  /// @brief Try to push an element (producer side).
  /// @param value The value to enqueue.
  /// @return true if successful, false if queue is full.
  bool try_push(const T & value) noexcept
  {
    const std::size_t current_head = head_.load(std::memory_order_relaxed);
    const std::size_t next_head = (current_head + 1) % (Capacity + 1);

    if (next_head == tail_.load(std::memory_order_acquire)) {
      return false;  // Full
    }

    buffer_[current_head] = value;
    head_.store(next_head, std::memory_order_release);
    return true;
  }

  /// @brief Try to push an element via move (producer side).
  /// @param value The value to enqueue (moved).
  /// @return true if successful, false if queue is full.
  bool try_push(T && value) noexcept
  {
    const std::size_t current_head = head_.load(std::memory_order_relaxed);
    const std::size_t next_head = (current_head + 1) % (Capacity + 1);

    if (next_head == tail_.load(std::memory_order_acquire)) {
      return false;
    }

    buffer_[current_head] = std::move(value);
    head_.store(next_head, std::memory_order_release);
    return true;
  }

  /// @brief Try to pop an element (consumer side).
  /// @return The dequeued value, or std::nullopt if empty.
  [[nodiscard]] std::optional<T> try_pop() noexcept
  {
    const std::size_t current_tail = tail_.load(std::memory_order_relaxed);

    if (current_tail == head_.load(std::memory_order_acquire)) {
      return std::nullopt;  // Empty
    }

    T value = std::move(buffer_[current_tail]);
    tail_.store((current_tail + 1) % (Capacity + 1), std::memory_order_release);
    return value;
  }

  /// @brief Check if the queue is empty.
  [[nodiscard]] bool empty() const noexcept
  {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  /// @brief Get the number of elements currently in the queue.
  [[nodiscard]] std::size_t size() const noexcept
  {
    const std::size_t h = head_.load(std::memory_order_acquire);
    const std::size_t t = tail_.load(std::memory_order_acquire);
    return (h >= t) ? (h - t) : (Capacity + 1 - t + h);
  }

  /// @brief Compile-time capacity.
  [[nodiscard]] static constexpr std::size_t capacity() noexcept
  {
    return Capacity;
  }

  /// @brief Reset the queue to empty state.
  /// @note Only safe when no concurrent push/pop is happening.
  void reset() noexcept
  {
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
  }

private:
  std::array<T, Capacity + 1> buffer_{};

  /// Cache-line aligned to prevent false sharing between producer and consumer.
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__LOCK_FREE_QUEUE_HPP_
