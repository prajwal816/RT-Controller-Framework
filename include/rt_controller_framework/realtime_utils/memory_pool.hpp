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

/// @file memory_pool.hpp
/// @brief Compile-time-sized pool of pre-allocated memory blocks.

#ifndef RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__MEMORY_POOL_HPP_
#define RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__MEMORY_POOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <new>

namespace rt_controller_framework
{
namespace realtime_utils
{

/// @brief O(1) alloc/dealloc pool via intrusive free-list.
/// @tparam T Block type.
/// @tparam PoolSize Number of blocks (compile-time constant).
template<typename T, std::size_t PoolSize>
class MemoryPool
{
public:
  MemoryPool()
  {
    reset();
  }

  /// @brief Allocate a block from the pool.
  /// @return Pointer to an initialized block, or nullptr if exhausted.
  T * allocate() noexcept
  {
    if (free_head_ == nullptr) {
      return nullptr;
    }

    FreeNode * node = free_head_;
    free_head_ = node->next;
    --available_;

    // Placement new to construct T in the raw storage
    T * ptr = reinterpret_cast<T *>(node);
    new (ptr) T();
    return ptr;
  }

  /// @brief Return a block to the pool.
  /// @param ptr Pointer previously obtained from allocate().
  void deallocate(T * ptr) noexcept
  {
    if (ptr == nullptr) {
      return;
    }

    // Bounds check: ensure the pointer belongs to this pool
    auto * raw = reinterpret_cast<uint8_t *>(ptr);
    auto * pool_start = reinterpret_cast<uint8_t *>(&storage_[0]);
    auto * pool_end = reinterpret_cast<uint8_t *>(&storage_[PoolSize]);

    if (raw < pool_start || raw >= pool_end) {
      return;  // Foreign pointer — ignore silently
    }

    // Destroy the object
    ptr->~T();

    // Return to free list
    auto * node = reinterpret_cast<FreeNode *>(ptr);
    node->next = free_head_;
    free_head_ = node;
    ++available_;
  }

  /// @brief Reset the pool (invalidates all outstanding pointers).
  void reset() noexcept
  {
    free_head_ = nullptr;
    available_ = PoolSize;

    for (std::size_t i = 0; i < PoolSize; ++i) {
      auto * node = reinterpret_cast<FreeNode *>(&storage_[i]);
      node->next = free_head_;
      free_head_ = node;
    }
  }

  /// @brief Number of currently available blocks.
  [[nodiscard]] std::size_t available() const noexcept
  {
    return available_;
  }

  /// @brief Compile-time pool capacity.
  [[nodiscard]] static constexpr std::size_t capacity() noexcept
  {
    return PoolSize;
  }

private:
  /// Intrusive free-list node.
  struct FreeNode
  {
    FreeNode * next{nullptr};
  };

  /// Ensure each block can hold either T or FreeNode.
  static constexpr std::size_t kBlockSize =
    sizeof(T) > sizeof(FreeNode) ? sizeof(T) : sizeof(FreeNode);
  static constexpr std::size_t kBlockAlign =
    alignof(T) > alignof(FreeNode) ? alignof(T) : alignof(FreeNode);

  struct alignas(kBlockAlign) Block
  {
    uint8_t data[kBlockSize];
  };

  std::array<Block, PoolSize> storage_{};
  FreeNode * free_head_{nullptr};
  std::size_t available_{0};
};

}  // namespace realtime_utils
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__REALTIME_UTILS__MEMORY_POOL_HPP_
