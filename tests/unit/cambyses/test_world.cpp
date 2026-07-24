#include <gtest/gtest.h>

#include "cambyses/world.hpp"

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

struct Tag {};

class WorldTest : public ::testing::Test {
protected:
    void SetUp() override {
        ComponentRegistry::register_component<Position>("Position");
        ComponentRegistry::register_component<Velocity>("Velocity");
        ComponentRegistry::register_component<Health>("Health");
        ComponentRegistry::register_component<Tag>("Tag");
    }
};

TEST_F(WorldTest, CreateEntity) {
    World world;
    Entity e = world.create();
    EXPECT_TRUE(world.is_valid(e));
    EXPECT_EQ(world.entity_count(), 1);
}

TEST_F(WorldTest, CreateMultipleEntities) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();
    EXPECT_TRUE(world.is_valid(e0));
    EXPECT_TRUE(world.is_valid(e1));
    EXPECT_EQ(world.entity_count(), 2);
}

TEST_F(WorldTest, DestroyEntity) {
    World world;
    Entity e = world.create();
    world.destroy(e);
    EXPECT_FALSE(world.is_valid(e));
    EXPECT_EQ(world.entity_count(), 0);
}

TEST_F(WorldTest, AddAndGetComponent) {
    World world;
    Entity e = world.create();
    world.add_component(e, Position{3.0f, 4.0f});

    EXPECT_TRUE(world.has_component<Position>(e));
    EXPECT_FLOAT_EQ(world.get_component<Position>(e).x, 3.0f);
    EXPECT_FLOAT_EQ(world.get_component<Position>(e).y, 4.0f);
}

TEST_F(WorldTest, RemoveComponent) {
    World world;
    Entity e = world.create();
    world.add_component(e, Position{1.0f, 2.0f});
    world.add_component(e, Velocity{0.1f, 0.2f});

    world.remove_component<Velocity>(e);
    EXPECT_TRUE(world.has_component<Position>(e));
    EXPECT_FALSE(world.has_component<Velocity>(e));
    EXPECT_FLOAT_EQ(world.get_component<Position>(e).x, 1.0f);
}

TEST_F(WorldTest, AddComponentMigratesEntity) {
    World world;
    Entity e = world.create();
    world.add_component(e, Position{1.0f, 2.0f});

    EXPECT_EQ(world.archetype_count(), 2);

    world.add_component(e, Velocity{0.1f, 0.2f});
    EXPECT_TRUE(world.has_component<Position>(e));
    EXPECT_TRUE(world.has_component<Velocity>(e));
    EXPECT_FLOAT_EQ(world.get_component<Position>(e).x, 1.0f);
    EXPECT_FLOAT_EQ(world.get_component<Velocity>(e).dx, 0.1f);
}

TEST_F(WorldTest, RemoveComponentMigratesEntity) {
    World world;
    Entity e = world.create();
    world.add_component(e, Position{1.0f, 2.0f});
    world.add_component(e, Velocity{0.1f, 0.2f});
    world.add_component(e, Health{100});

    world.remove_component<Velocity>(e);
    EXPECT_TRUE(world.has_component<Position>(e));
    EXPECT_FALSE(world.has_component<Velocity>(e));
    EXPECT_TRUE(world.has_component<Health>(e));
}

TEST_F(WorldTest, DestroyEntityWithComponents) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();

    world.add_component(e0, Position{1.0f, 0.0f});
    world.add_component(e0, Health{100});
    world.add_component(e1, Position{2.0f, 0.0f});

    world.destroy(e0);
    EXPECT_FALSE(world.is_valid(e0));
    EXPECT_TRUE(world.is_valid(e1));
    EXPECT_TRUE(world.has_component<Position>(e1));

    EXPECT_FLOAT_EQ(world.get_component<Position>(e1).x, 2.0f);
}

TEST_F(WorldTest, DestroyMiddleEntityPreservesOthers) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();
    Entity e2 = world.create();

    world.add_component(e0, Position{1.0f, 0.0f});
    world.add_component(e1, Position{2.0f, 0.0f});
    world.add_component(e2, Position{3.0f, 0.0f});

    world.destroy(e1);
    EXPECT_EQ(world.entity_count(), 2);

    foundation::size_t count = 0;
    float sum = 0.0f;
    world.query<Position>().each([&](Entity, Position& pos) {
        ++count;
        sum += pos.x;
    });
    EXPECT_EQ(count, 2);
    EXPECT_FLOAT_EQ(sum, 4.0f);
}

TEST_F(WorldTest, QueryIntegration) {
    World world;
    for (int i = 0; i < 10; ++i) {
        Entity e = world.create();
        world.add_component(e, Position{static_cast<float>(i), 0.0f});
        if (i % 2 == 0) {
            world.add_component(e, Velocity{1.0f, 0.0f});
        }
    }

    foundation::size_t moving_count = 0;
    world.query<Position, Velocity>().each([&](Entity, Position& pos, const Velocity& vel) {
        pos.x += vel.dx;
        ++moving_count;
    });
    EXPECT_EQ(moving_count, 5);

    EXPECT_FLOAT_EQ(world.get_component<Position>(Entity{0, 0}).x, 1.0f);
    EXPECT_FLOAT_EQ(world.get_component<Position>(Entity{1, 0}).x, 1.0f);
    EXPECT_FLOAT_EQ(world.get_component<Position>(Entity{2, 0}).x, 3.0f);
}

TEST_F(WorldTest, QueryWithExclude) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();
    Entity e2 = world.create();

    world.add_component(e0, Position{1.0f, 0.0f});
    world.add_component(e0, Tag{});

    world.add_component(e1, Position{2.0f, 0.0f});

    world.add_component(e2, Position{3.0f, 0.0f});
    world.add_component(e2, Tag{});

    foundation::size_t count = 0;
    float sum = 0.0f;
    world.query<Position>().exclude<Tag>().each([&](Entity, Position& pos) {
        ++count;
        sum += pos.x;
    });
    EXPECT_EQ(count, 1);
    EXPECT_FLOAT_EQ(sum, 2.0f);
}

TEST_F(WorldTest, ManyEntities) {
    World world;
    const foundation::u32 N = 10000;
    foundation::DynamicArray<Entity> entities;
    entities.reserve(N);

    for (foundation::u32 i = 0; i < N; ++i) {
        Entity e = world.create();
        world.add_component(e, Position{static_cast<float>(i), 0.0f});
        entities.push_back(e);
    }

    EXPECT_EQ(world.entity_count(), N);

    foundation::size_t count = 0;
    world.query<Position>().each([&](Entity, Position&) { ++count; });
    EXPECT_EQ(count, N);

    for (foundation::u32 i = 0; i < N / 2; ++i) {
        world.destroy(entities[i]);
    }
    EXPECT_EQ(world.entity_count(), N / 2);
}

TEST_F(WorldTest, EventIntegration) {
    World world;
    int event_count = 0;

    world.subscribe<Health>([&](const Health&) { ++event_count; });
    world.publish(Health{50});
    EXPECT_EQ(event_count, 1);
}

TEST_F(WorldTest, SystemIntegration) {
    World world;
    Entity e = world.create();
    world.add_component(e, Position{0.0f, 0.0f});
    world.add_component(e, Velocity{1.0f, 1.0f});

    world.add_system([](World& w) {
        w.query<Position, Velocity>().each([](Entity, Position& pos, const Velocity& vel) {
            pos.x += vel.dx;
            pos.y += vel.dy;
        });
    });

    world.run_systems();
    EXPECT_FLOAT_EQ(world.get_component<Position>(e).x, 1.0f);
    EXPECT_FLOAT_EQ(world.get_component<Position>(e).y, 1.0f);
}

TEST_F(WorldTest, RecycledEntityWorks) {
    World world;
    Entity e0 = world.create();
    world.add_component(e0, Position{1.0f, 2.0f});
    world.destroy(e0);

    Entity e1 = world.create();
    EXPECT_TRUE(world.is_valid(e1));
    EXPECT_FALSE(world.has_component<Position>(e1));
    world.add_component(e1, Position{5.0f, 6.0f});
    EXPECT_FLOAT_EQ(world.get_component<Position>(e1).x, 5.0f);
}

} // namespace
