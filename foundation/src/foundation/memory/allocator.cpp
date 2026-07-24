#include "foundation/memory/allocator.hpp"

#include "foundation/core/platform.hpp"

#include <cstdlib>
#include <cstring>

namespace foundation {

DefaultAllocator::DefaultAllocator(DefaultAllocator&&) noexcept = default;
DefaultAllocator& DefaultAllocator::operator=(DefaultAllocator&&) noexcept = default;

void* DefaultAllocator::allocate(size_t size, size_t alignment) {
    // posix_memalign requires alignment to be a multiple of sizeof(void*)
    constexpr size_t kMinAlignment = alignof(std::max_align_t);
    if (alignment < kMinAlignment) {
        alignment = kMinAlignment;
    }
    void* ptr = nullptr;
#if defined(PREONX_PLATFORM_WINDOWS)
    ptr = ::_aligned_malloc(size, alignment);
#else
    if (::posix_memalign(&ptr, alignment, size) != 0) {
        ptr = nullptr;
    }
#endif
    if (ptr) {
        total_allocated_ += size;
        ++allocation_count_;
    }
    return ptr;
}

void DefaultAllocator::deallocate(void* ptr, size_t size, size_t /*alignment*/) {
    if (ptr) {
#if defined(PREONX_PLATFORM_WINDOWS)
        ::_aligned_free(ptr);
#else
        ::free(ptr);
#endif
        if (total_allocated_ >= size) {
            total_allocated_ -= size;
        }
    }
}

DefaultAllocator& DefaultAllocator::instance() {
    static DefaultAllocator instance;
    return instance;
}

Allocator& default_allocator() {
    return DefaultAllocator::instance();
}

void* memory_allocate(size_t size, size_t alignment) {
    return default_allocator().allocate(size, alignment);
}

void memory_deallocate(void* ptr, size_t size, size_t alignment) {
    default_allocator().deallocate(ptr, size, alignment);
}

void* memory_allocate_zero(size_t size, size_t alignment) {
    void* ptr = memory_allocate(size, alignment);
    if (ptr) {
        std::memset(ptr, 0, size);
    }
    return ptr;
}

} // namespace foundation
