#include <gtest/gtest.h>

#include "foundation/hashing/hash.hpp"

#include <cstring>
#include <string>

namespace {

TEST(HashTest, Fnv1aConsistency) {
    const char* data = "hello world";
    foundation::HashValue h1 = foundation::hash::fnv1a(data, std::strlen(data));
    foundation::HashValue h2 = foundation::hash::fnv1a(data, std::strlen(data));
    EXPECT_EQ(h1, h2);
}

TEST(HashTest, DifferentInputsDifferentHashes) {
    const char* a = "hello";
    const char* b = "world";
    foundation::HashValue h1 = foundation::hash::fnv1a(a, std::strlen(a));
    foundation::HashValue h2 = foundation::hash::fnv1a(b, std::strlen(b));
    EXPECT_NE(h1, h2);
}

TEST(HashTest, HashBytes) {
    int x = 42;
    foundation::HashValue h = foundation::hash::hash_bytes(&x, sizeof(x));
    EXPECT_NE(h, 0u);
}

TEST(HashTest, HashString) {
    foundation::HashValue h = foundation::hash::hash_string("test string");
    EXPECT_NE(h, 0u);

    foundation::HashValue h2 = foundation::hash::hash_string("test string");
    EXPECT_EQ(h, h2);
}

TEST(HashTest, HashValueForStruct) {
    struct Vec2 {
        float x, y;
    };
    Vec2 v{1.0f, 2.0f};
    foundation::HashValue h = foundation::hash::hash_value(v);
    EXPECT_NE(h, 0u);
}

TEST(HashTest, CombineProducesNonZero) {
    foundation::HashValue h = foundation::hash::combine({100, 200, 300});
    EXPECT_NE(h, 0u);
}

TEST(HashTest, CombineOrderMatters) {
    foundation::HashValue h1 = foundation::hash::combine({100, 200});
    foundation::HashValue h2 = foundation::hash::combine({200, 100});
    EXPECT_NE(h1, h2);
}

TEST(HashTest, EmptyInput) {
    foundation::HashValue h = foundation::hash::fnv1a(nullptr, 0);
    // Should not crash, returns consistent value
    foundation::HashValue h2 = foundation::hash::fnv1a(nullptr, 0);
    EXPECT_EQ(h, h2);
}

TEST(HashTest, LargeInput) {
    std::string large(10000, 'a');
    foundation::HashValue h = foundation::hash::hash_string(large);
    EXPECT_NE(h, 0u);

    foundation::HashValue h2 = foundation::hash::hash_string(large);
    EXPECT_EQ(h, h2);
}

} // namespace
