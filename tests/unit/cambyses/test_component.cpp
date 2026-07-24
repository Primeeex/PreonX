#include <gtest/gtest.h>

#include "cambyses/component/component_registry.hpp"

using namespace cambyses;

namespace {

struct Position {
    float x = 0.0f;
    float y = 0.0f;
};

struct Velocity {
    float dx = 0.0f;
    float dy = 0.0f;
};

struct Health {
    int value = 100;
};

TEST(ComponentRegistryTest, TypeIdsAreUnique) {
    EXPECT_NE(ComponentRegistry::type_id<Position>(), ComponentRegistry::type_id<Velocity>());
    EXPECT_NE(ComponentRegistry::type_id<Velocity>(), ComponentRegistry::type_id<Health>());
    EXPECT_NE(ComponentRegistry::type_id<Position>(), ComponentRegistry::type_id<Health>());
}

TEST(ComponentRegistryTest, TypeIdsAreStable) {
    auto id1 = ComponentRegistry::type_id<Position>();
    auto id2 = ComponentRegistry::type_id<Position>();
    EXPECT_EQ(id1, id2);
}

TEST(ComponentRegistryTest, RegisterComponent) {
    ComponentRegistry::register_component<Position>("Position");
    EXPECT_TRUE(ComponentRegistry::is_registered(ComponentRegistry::type_id<Position>()));

    const auto& info = ComponentRegistry::get_info(ComponentRegistry::type_id<Position>());
    EXPECT_EQ(info.size, sizeof(Position));
    EXPECT_EQ(info.alignment, alignof(Position));
    EXPECT_STREQ(info.name, "Position");
}

TEST(ComponentRegistryTest, ColumnOpsDefaultConstruct) {
    auto ops = ColumnOps::for_type<Position>();
    alignas(Position) unsigned char buffer[sizeof(Position)];
    ops.default_construct(buffer);
    auto* pos = reinterpret_cast<Position*>(buffer);
    EXPECT_FLOAT_EQ(pos->x, 0.0f);
    EXPECT_FLOAT_EQ(pos->y, 0.0f);
    ops.destruct(buffer);
}

TEST(ComponentRegistryTest, ColumnOpsCopyConstruct) {
    auto ops = ColumnOps::for_type<Health>();
    alignas(Health) unsigned char src_buf[sizeof(Health)];
    ::new (src_buf) Health{42};

    alignas(Health) unsigned char dst_buf[sizeof(Health)];
    ops.copy_construct(dst_buf, src_buf);
    auto* dst = reinterpret_cast<Health*>(dst_buf);
    EXPECT_EQ(dst->value, 42);

    ops.destruct(dst_buf);
    ops.destruct(src_buf);
}

TEST(ComponentRegistryTest, ColumnOpsMoveConstruct) {
    auto ops = ColumnOps::for_type<Health>();
    alignas(Health) unsigned char src_buf[sizeof(Health)];
    ::new (src_buf) Health{99};

    alignas(Health) unsigned char dst_buf[sizeof(Health)];
    ops.move_construct(dst_buf, src_buf);
    auto* dst = reinterpret_cast<Health*>(dst_buf);
    EXPECT_EQ(dst->value, 99);

    ops.destruct(dst_buf);
}

TEST(ComponentRegistryTest, TriviallyCopyableDetection) {
    struct TrivialPos {
        float x;
        float y;
    };
    auto ops = ColumnOps::for_type<TrivialPos>();
    EXPECT_TRUE(ops.trivial_copy);
    EXPECT_TRUE(ops.trivial_destruct);
    EXPECT_TRUE(ops.trivial_construct);
}

} // namespace
