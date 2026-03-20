/**
 * @file lock_free_queue.hpp
 * @brief Single-Producer Single-Consumer (SPSC) lock-free ring buffer.
 *
 * Designed for real-time systems where dynamic memory allocation is
 * prohibited in the hot path. Uses atomic operations with acquire/release
 * memory ordering for thread-safe, wait-free data transfer between
 * the RT control loop and non-RT threads.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__LOCK_FREE_QUEUE_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__LOCK_FREE_QUEUE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace rt_controller_framework {
namespace realtime_utils {

/**
 * @brief A fixed-capacity, lock-free SPSC ring buffer.
 *
 * @tparam T       Element type (must be trivially copyable for best RT behavior).
 * @tparam Capacity Maximum number of elements the queue can hold. Must be > 0.
 *
 * Thread-safety guarantee:
 *  - Exactly ONE producer thread may call try_push().
 *  - Exactly ONE consumer thread may call try_pop().
 *  - No mutexes, no dynamic allocation, no exceptions.
 */
template <typename T, std::size_t Capacity>
class LockFreeQueue {
  static_assert(Capacity > 0, "LockFreeQueue capacity must be > 0");

 public:
  LockFreeQueue() : head_(0), tail_(0) {}

  // Non-copyable, non-movable (atomic members)
  LockFreeQueue(const LockFreeQueue&) = delete;
  LockFreeQueue& operator=(const LockFreeQueue&) = delete;
  LockFreeQueue(LockFreeQueue&&) = delete;
  LockFreeQueue& operator=(LockFreeQueue&&) = delete;

  /**
   * @brief Attempt to push an element into the queue.
   *
   * @param value The value to enqueue.
   * @return true  if the element was successfully enqueued.
   * @return false if the queue is full.
   *
   * @note Only the producer thread may call this method.
   */
  bool try_push(const T& value) noexcept {
    const std::size_t current_tail = tail_.load(std::memory_order_relaxed);
    const std::size_t next_tail = increment(current_tail);

    if (next_tail == head_.load(std::memory_order_acquire)) {
      return false;  // Queue is full
    }

    buffer_[current_tail] = value;
    tail_.store(next_tail, std::memory_order_release);
    return true;
  }

  /**
   * @brief Attempt to pop an element from the queue.
   *
   * @return std::optional<T> containing the dequeued value, or std::nullopt if empty.
   *
   * @note Only the consumer thread may call this method.
   */
  std::optional<T> try_pop() noexcept {
    const std::size_t current_head = head_.load(std::memory_order_relaxed);

    if (current_head == tail_.load(std::memory_order_acquire)) {
      return std::nullopt;  // Queue is empty
    }

    T value = buffer_[current_head];
    head_.store(increment(current_head), std::memory_order_release);
    return value;
  }

  /**
   * @brief Check if the queue is empty.
   * @return true if the queue contains no elements.
   */
  [[nodiscard]] bool empty() const noexcept {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  /**
   * @brief Get the current number of elements in the queue.
   * @return Approximate size (may be slightly stale in concurrent use).
   */
  [[nodiscard]] std::size_t size() const noexcept {
    const std::size_t h = head_.load(std::memory_order_acquire);
    const std::size_t t = tail_.load(std::memory_order_acquire);
    if (t >= h) {
      return t - h;
    }
    return kBufferSize - h + t;
  }

  /**
   * @brief Get the maximum capacity of the queue.
   * @return The compile-time capacity.
   */
  static constexpr std::size_t capacity() noexcept { return Capacity; }

  /**
   * @brief Reset the queue to empty state.
   * @warning NOT thread-safe. Only call when no concurrent access occurs.
   */
  void reset() noexcept {
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
  }

 private:
  static constexpr std::size_t kBufferSize = Capacity + 1;  // +1 for sentinel

  [[nodiscard]] static constexpr std::size_t increment(std::size_t idx) noexcept {
    return (idx + 1) % kBufferSize;
  }

  std::array<T, kBufferSize> buffer_{};
  alignas(64) std::atomic<std::size_t> head_;  // Consumer index
  alignas(64) std::atomic<std::size_t> tail_;  // Producer index
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__LOCK_FREE_QUEUE_HPP_
