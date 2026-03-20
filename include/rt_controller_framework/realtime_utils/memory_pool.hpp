/**
 * @file memory_pool.hpp
 * @brief Pre-allocated fixed-size memory block pool for real-time systems.
 *
 * Provides O(1) allocation and deallocation via a free-list of pre-allocated
 * blocks. No calls to new/malloc/free/delete occur after construction,
 * making this safe for use in hard real-time control loops.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__MEMORY_POOL_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__MEMORY_POOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>

namespace rt_controller_framework {
namespace realtime_utils {

/**
 * @brief A compile-time-sized pool of memory blocks.
 *
 * @tparam T        The object type to pool.
 * @tparam PoolSize Maximum number of objects in the pool.
 *
 * Usage:
 * @code
 *   MemoryPool<MyStruct, 128> pool;
 *   auto* ptr = pool.allocate();
 *   if (ptr) { ... }
 *   pool.deallocate(ptr);
 * @endcode
 *
 * @note NOT thread-safe. If concurrent access is needed, guard externally
 *       or use one pool per thread.
 */
template <typename T, std::size_t PoolSize>
class MemoryPool {
  static_assert(PoolSize > 0, "MemoryPool size must be > 0");

 public:
  MemoryPool() { reset(); }

  ~MemoryPool() = default;

  // Non-copyable, non-movable
  MemoryPool(const MemoryPool&) = delete;
  MemoryPool& operator=(const MemoryPool&) = delete;
  MemoryPool(MemoryPool&&) = delete;
  MemoryPool& operator=(MemoryPool&&) = delete;

  /**
   * @brief Allocate a block from the pool.
   *
   * @return Pointer to an uninitialized block, or nullptr if the pool is
   *         exhausted. The caller is responsible for constructing the object
   *         via placement new if needed.
   *
   * @note O(1) — pops from the head of the free-list.
   */
  T* allocate() noexcept {
    if (free_head_ == nullptr) {
      return nullptr;  // Pool exhausted
    }
    FreeNode* node = free_head_;
    free_head_ = node->next;
    --free_count_;
    return reinterpret_cast<T*>(node);
  }

  /**
   * @brief Return a block to the pool.
   *
   * @param ptr Pointer previously obtained from allocate().
   *
   * @note O(1) — pushes onto the head of the free-list.
   * @warning Passing a pointer not from this pool or double-freeing
   *          results in undefined behavior.
   */
  void deallocate(T* ptr) noexcept {
    if (ptr == nullptr) return;

    // Validate that ptr belongs to our storage
    auto* raw = reinterpret_cast<std::uint8_t*>(ptr);
    auto* storage_start = reinterpret_cast<std::uint8_t*>(&storage_[0]);
    auto* storage_end = storage_start + sizeof(storage_);

    if (raw < storage_start || raw >= storage_end) {
      return;  // Not our pointer — silently ignore in RT context
    }

    auto* node = reinterpret_cast<FreeNode*>(ptr);
    node->next = free_head_;
    free_head_ = node;
    ++free_count_;
  }

  /**
   * @brief Reset the pool to its initial state (all blocks free).
   * @warning NOT safe to call while allocated blocks are in use.
   */
  void reset() noexcept {
    free_head_ = nullptr;
    free_count_ = PoolSize;

    for (std::size_t i = 0; i < PoolSize; ++i) {
      auto* node = reinterpret_cast<FreeNode*>(&storage_[i]);
      node->next = free_head_;
      free_head_ = node;
    }
  }

  /** @brief Number of currently available (free) blocks. */
  [[nodiscard]] std::size_t available() const noexcept { return free_count_; }

  /** @brief Total pool capacity. */
  static constexpr std::size_t capacity() noexcept { return PoolSize; }

  /** @brief Number of currently allocated blocks. */
  [[nodiscard]] std::size_t in_use() const noexcept {
    return PoolSize - free_count_;
  }

 private:
  struct FreeNode {
    FreeNode* next;
  };

  // Ensure each block is large enough to hold either T or a FreeNode pointer
  static constexpr std::size_t kBlockSize =
      sizeof(T) > sizeof(FreeNode) ? sizeof(T) : sizeof(FreeNode);
  static constexpr std::size_t kBlockAlign =
      alignof(T) > alignof(FreeNode) ? alignof(T) : alignof(FreeNode);

  struct alignas(kBlockAlign) Block {
    std::uint8_t data[kBlockSize];
  };

  std::array<Block, PoolSize> storage_{};
  FreeNode* free_head_{nullptr};
  std::size_t free_count_{0};
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__MEMORY_POOL_HPP_
