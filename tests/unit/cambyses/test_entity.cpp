#include <gtest/gtest.h>

#include "cambyses/entity/entity_manager.hpp"

using namespace cambyses;
using foundation::u32;

namespace {

TEST(EntityManagerTest, CreateReturnsValidEntity) {
    EntityManager mgr;
    Entity e = mgr.create();
    EXPECT_TRUE(mgr.is_valid(e));
    EXPECT_EQ(e.index, 0);
    EXPECT_EQ(e.generation, 0);
}

TEST(EntityManagerTest, CreateMultipleEntities) {
    EntityManager mgr;
    Entity e0 = mgr.create();
    Entity e1 = mgr.create();
    Entity e2 = mgr.create();

    EXPECT_NE(e0.index, e1.index);
    EXPECT_NE(e1.index, e2.index);
    EXPECT_TRUE(mgr.is_valid(e0));
    EXPECT_TRUE(mgr.is_valid(e1));
    EXPECT_TRUE(mgr.is_valid(e2));
    EXPECT_EQ(mgr.alive_count(), 3);
}

TEST(EntityManagerTest, DestroyInvalidatesEntity) {
    EntityManager mgr;
    Entity e = mgr.create();
    mgr.destroy(e);
    EXPECT_FALSE(mgr.is_valid(e));
    EXPECT_EQ(mgr.alive_count(), 0);
}

TEST(EntityManagerTest, DestroyIncrementsGeneration) {
    EntityManager mgr;
    Entity e0 = mgr.create();
    mgr.destroy(e0);
    Entity e1 = mgr.create();
    EXPECT_EQ(e1.index, e0.index);
    EXPECT_GT(e1.generation, e0.generation);
    EXPECT_TRUE(mgr.is_valid(e1));
    EXPECT_FALSE(mgr.is_valid(e0));
}

TEST(EntityManagerTest, RecycledEntityIsValid) {
    EntityManager mgr;
    Entity e0 = mgr.create();
    mgr.destroy(e0);
    Entity e1 = mgr.create();
    EXPECT_TRUE(mgr.is_valid(e1));
    EXPECT_EQ(mgr.alive_count(), 1);
}

TEST(EntityManagerTest, StaleEntityIsInvalid) {
    EntityManager mgr;
    Entity e0 = mgr.create();
    Entity e1 = mgr.create();
    mgr.destroy(e0);
    Entity e2 = mgr.create();

    EXPECT_FALSE(mgr.is_valid(e0));
    EXPECT_TRUE(mgr.is_valid(e1));
    EXPECT_TRUE(mgr.is_valid(e2));
}

TEST(EntityManagerTest, NullEntityIsInvalid) {
    EntityManager mgr;
    EXPECT_FALSE(mgr.is_valid(kNullEntity));
}

TEST(EntityManagerTest, OutOfBoundsIsInvalid) {
    EntityManager mgr;
    Entity bad{999, 0};
    EXPECT_FALSE(mgr.is_valid(bad));
}

TEST(EntityManagerTest, MultipleDestroyAndCreate) {
    EntityManager mgr;
    for (int i = 0; i < 100; ++i) {
        (void)mgr.create();
    }
    EXPECT_EQ(mgr.alive_count(), 100);

    for (int i = 0; i < 50; ++i) {
        mgr.destroy(Entity{static_cast<u32>(i), 0});
    }
    EXPECT_EQ(mgr.alive_count(), 50);

    for (int i = 0; i < 50; ++i) {
        (void)mgr.create();
    }
    EXPECT_EQ(mgr.alive_count(), 100);
}

} // namespace
