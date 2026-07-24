#include <gtest/gtest.h>

#include "cambyses/archetype/archetype.hpp"
#include "cambyses/component/component_registry.hpp"
#include "cambyses/core/types.hpp"

using namespace cambyses;
using foundation::u32;
using foundation::size_t;

namespace {

struct Pos {
    float x = 0.0f;
    float y = 0.0f;
};

struct Vel {
    float dx = 1.0f;
    float dy = 2.0f;
};

struct HP {
    int value = 100;
};

class ArchetypeTest : public ::testing::Test {
protected:
    void SetUp() override {
        ComponentRegistry::register_component<Pos>("Pos");
        ComponentRegistry::register_component<Vel>("Vel");
        ComponentRegistry::register_component<HP>("HP");
    }
};

TEST_F(ArchetypeTest, CreateEmptyArchetype) {
    foundation::DynamicArray<ComponentTypeId> types;
    foundation::DynamicArray<ColumnOps> ops;
    Archetype arch(std::move(types), std::move(ops));
    EXPECT_TRUE(arch.empty());
    EXPECT_EQ(arch.size(), 0);
}

TEST_F(ArchetypeTest, AddEntity) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<Pos>(),
                                                       ComponentRegistry::type_id<Vel>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<Pos>(), ColumnOps::for_type<Vel>()};
    Archetype arch(std::move(types), std::move(ops));

    Entity e0{0, 0};
    Entity e1{1, 0};

    foundation::size_t row0 = arch.add_entity(e0);
    foundation::size_t row1 = arch.add_entity(e1);

    EXPECT_EQ(row0, 0);
    EXPECT_EQ(row1, 1);
    EXPECT_EQ(arch.size(), 2);
    EXPECT_EQ(arch.entity(0), e0);
    EXPECT_EQ(arch.entity(1), e1);
}

TEST_F(ArchetypeTest, SetAndGetComponents) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<Pos>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<Pos>()};
    Archetype arch(std::move(types), std::move(ops));

    Entity e0{0, 0};
    foundation::size_t row = arch.add_entity(e0);

    arch.set_component(row, Pos{3.0f, 4.0f});
    EXPECT_FLOAT_EQ(arch.get_component<Pos>(row).x, 3.0f);
    EXPECT_FLOAT_EQ(arch.get_component<Pos>(row).y, 4.0f);
}

TEST_F(ArchetypeTest, RemoveEntitySwapLast) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<Pos>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<Pos>()};
    Archetype arch(std::move(types), std::move(ops));

    Entity e0{0, 0};
    Entity e1{1, 0};
    Entity e2{2, 0};

    (void)arch.add_entity(e0);
    (void)arch.add_entity(e1);
    (void)arch.add_entity(e2);

    arch.set_component(0, Pos{10.0f, 0.0f});
    arch.set_component(1, Pos{20.0f, 0.0f});
    arch.set_component(2, Pos{30.0f, 0.0f});

    Entity swapped = arch.remove_entity(1);
    EXPECT_EQ(swapped, e2);
    EXPECT_EQ(arch.size(), 2);
    EXPECT_EQ(arch.entity(0), e0);
    EXPECT_EQ(arch.entity(1), e2);

    EXPECT_FLOAT_EQ(arch.get_component<Pos>(1).x, 30.0f);
}

TEST_F(ArchetypeTest, RemoveLastEntity) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<HP>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<HP>()};
    Archetype arch(std::move(types), std::move(ops));

    Entity e0{0, 0};
    Entity e1{1, 0};
    (void)arch.add_entity(e0);
    (void)arch.add_entity(e1);

    Entity swapped = arch.remove_entity(1);
    EXPECT_EQ(swapped, kNullEntity);
    EXPECT_EQ(arch.size(), 1);
    EXPECT_EQ(arch.entity(0), e0);
}

TEST_F(ArchetypeTest, HasComponent) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<Pos>(),
                                                       ComponentRegistry::type_id<Vel>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<Pos>(), ColumnOps::for_type<Vel>()};
    Archetype arch(std::move(types), std::move(ops));

    EXPECT_TRUE(arch.has_component<Pos>());
    EXPECT_TRUE(arch.has_component<Vel>());
    EXPECT_FALSE(arch.has_component<HP>());
}

TEST_F(ArchetypeTest, ColumnIndexFor) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<HP>(),
                                                       ComponentRegistry::type_id<Pos>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<HP>(), ColumnOps::for_type<Pos>()};
    Archetype arch(std::move(types), std::move(ops));

    EXPECT_EQ(arch.column_index_for(ComponentRegistry::type_id<HP>()), 0);
    EXPECT_EQ(arch.column_index_for(ComponentRegistry::type_id<Pos>()), 1);
    EXPECT_EQ(arch.column_index_for(ComponentRegistry::type_id<Vel>()), foundation::kNotFound);
}

TEST_F(ArchetypeTest, IterationOverEntities) {
    foundation::DynamicArray<ComponentTypeId> types = {ComponentRegistry::type_id<Pos>()};
    foundation::DynamicArray<ColumnOps> ops = {ColumnOps::for_type<Pos>()};
    Archetype arch(std::move(types), std::move(ops));

    for (int i = 0; i < 10; ++i) {
        Entity e{static_cast<u32>(i), 0};
        foundation::size_t row = arch.add_entity(e);
        arch.set_component(row, Pos{static_cast<float>(i), 0.0f});
    }

    float sum = 0.0f;
    for (foundation::size_t i = 0; i < arch.size(); ++i) {
        sum += arch.get_component<Pos>(i).x;
    }
    EXPECT_FLOAT_EQ(sum, 45.0f);
}

} // namespace
