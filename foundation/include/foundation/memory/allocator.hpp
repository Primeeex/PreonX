#pragma once

#include "foundation/core/types.hpp"

#include <cstddef>
#include <memory>

namespace foundation {

class Allocator {
public:
    virtual ~Allocator() = default;

    [[nodiscard]] virtual void* allocate(size_t size, size_t alignment = alignof(std::max_align_t)) = 0;
    virtual void deallocate(void* ptr, size_t size, size_t alignment = alignof(std::max_align_t)) = 0;

    [[nodiscard]] virtual const char* name() const noexcept = 0;
    [[nodiscard]] virtual size_t total_allocated() const noexcept = 0;
    [[nodiscard]] virtual size_t allocation_count() const noexcept = 0;
};

class DefaultAllocator : public Allocator {
public:
    DefaultAllocator() = default;
    ~DefaultAllocator() override = default;

    DefaultAllocator(const DefaultAllocator&) = delete;
    DefaultAllocator& operator=(const DefaultAllocator&) = delete;
    DefaultAllocator(DefaultAllocator&&) noexcept;
    DefaultAllocator& operator=(DefaultAllocator&&) noexcept;

    [[nodiscard]] void* allocate(size_t size, size_t alignment = alignof(std::max_align_t)) override;
    void deallocate(void* ptr, size_t size, size_t alignment = alignof(std::max_align_t)) override;

    [[nodiscard]] const char* name() const noexcept override {
        return "DefaultAllocator";
    }

    [[nodiscard]] size_t total_allocated() const noexcept override {
        return total_allocated_;
    }

    [[nodiscard]] size_t allocation_count() const noexcept override {
        return allocation_count_;
    }

    [[nodiscard]] static DefaultAllocator& instance();

private:
    size_t total_allocated_ = 0;
    size_t allocation_count_ = 0;
};

/// Returns the global default allocator.
[[nodiscard]] Allocator& default_allocator();

/// Allocate memory using the default allocator.
[[nodiscard]] void* memory_allocate(size_t size, size_t alignment = alignof(std::max_align_t));

/// Deallocate memory using the default allocator.
void memory_deallocate(void* ptr, size_t size, size_t alignment = alignof(std::max_align_t));

/// Allocate and zero-initialize memory.
[[nodiscard]] void* memory_allocate_zero(size_t size, size_t alignment = alignof(std::max_align_t));

} // namespace foundation
