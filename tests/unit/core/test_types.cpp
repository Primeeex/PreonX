#include <gtest/gtest.h>

#include "foundation/core/types.hpp"

namespace {

TEST(TypesTest, TypeAliases) {
    // Verify type aliases are the correct sizes
    EXPECT_EQ(sizeof(foundation::i8), 1u);
    EXPECT_EQ(sizeof(foundation::i16), 2u);
    EXPECT_EQ(sizeof(foundation::i32), 4u);
    EXPECT_EQ(sizeof(foundation::i64), 8u);

    EXPECT_EQ(sizeof(foundation::u8), 1u);
    EXPECT_EQ(sizeof(foundation::u16), 2u);
    EXPECT_EQ(sizeof(foundation::u32), 4u);
    EXPECT_EQ(sizeof(foundation::u64), 8u);

    EXPECT_EQ(sizeof(foundation::f32), 4u);
    EXPECT_EQ(sizeof(foundation::f64), 8u);
}

TEST(TypesTest, Constants) {
    EXPECT_EQ(foundation::kMaxU8, 255u);
    EXPECT_EQ(foundation::kMaxU16, 65535u);
    EXPECT_EQ(foundation::kMaxU32, 4294967295u);

    EXPECT_EQ(foundation::kMinI8, -128);
    EXPECT_EQ(foundation::kMaxI8, 127);
    EXPECT_EQ(foundation::kMinI32, -2147483648);
    EXPECT_EQ(foundation::kMaxI32, 2147483647);
}

TEST(TypesTest, NotFoundIsMax) {
    EXPECT_EQ(foundation::kNotFound, static_cast<size_t>(-1));
}

} // namespace
