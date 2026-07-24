#include <gtest/gtest.h>

#include "foundation/memory/allocator.hpp"

namespace {

TEST(DefaultAllocatorTest, Instance) {
    foundation::DefaultAllocator& alloc = foundation::DefaultAllocator::instance();
    EXPECT_STREQ(alloc.name(), "DefaultAllocator");
}

TEST(DefaultAllocatorTest, AllocateDeallocate) {
    foundation::DefaultAllocator alloc;
    void* ptr = alloc.allocate(64);
    EXPECT_NE(ptr, nullptr);
    alloc.deallocate(ptr, 64);
}

TEST(DefaultAllocatorTest, Alignment) {
    foundation::DefaultAllocator alloc;
    void* ptr = alloc.allocate(64, 64);
    EXPECT_NE(ptr, nullptr);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(ptr) % 64, 0u);
    alloc.deallocate(ptr, 64, 64);
}

TEST(DefaultAllocatorTest, TrackAllocations) {
    foundation::DefaultAllocator alloc;
    EXPECT_EQ(alloc.total_allocated(), 0u);
    EXPECT_EQ(alloc.allocation_count(), 0u);

    void* ptr = alloc.allocate(128);
    EXPECT_EQ(alloc.total_allocated(), 128u);
    EXPECT_EQ(alloc.allocation_count(), 1u);

    alloc.deallocate(ptr, 128);
}

TEST(DefaultAllocatorTest, MultipleAllocations) {
    foundation::DefaultAllocator alloc;
    void* p1 = alloc.allocate(64);
    void* p2 = alloc.allocate(128);
    void* p3 = alloc.allocate(256);

    EXPECT_NE(p1, nullptr);
    EXPECT_NE(p2, nullptr);
    EXPECT_NE(p3, nullptr);
    EXPECT_NE(p1, p2);
    EXPECT_NE(p2, p3);

    alloc.deallocate(p1, 64);
    alloc.deallocate(p2, 128);
    alloc.deallocate(p3, 256);
}

TEST(DefaultAllocatorTest, GlobalFunctions) {
    void* ptr = foundation::memory_allocate(128);
    EXPECT_NE(ptr, nullptr);
    foundation::memory_deallocate(ptr, 128);

    void* zeroed = foundation::memory_allocate_zero(64);
    EXPECT_NE(zeroed, nullptr);
    const auto* bytes = static_cast<const foundation::u8*>(zeroed);
    for (size_t i = 0; i < 64; ++i) {
        EXPECT_EQ(bytes[i], 0u);
    }
    foundation::memory_deallocate(zeroed, 64);
}

TEST(DefaultAllocatorTest, LargeAllocation) {
    foundation::DefaultAllocator alloc;
    void* ptr = alloc.allocate(1024 * 1024); // 1 MB
    EXPECT_NE(ptr, nullptr);
    alloc.deallocate(ptr, 1024 * 1024);
}

} // namespace
