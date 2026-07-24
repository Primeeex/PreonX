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

struct Tag {};

class QueryTest : public ::testing::Test {
protected:
    void SetUp() override {
        ComponentRegistry::register_component<Position>("Position");
        ComponentRegistry::register_component<Velocity>("Velocity");
        ComponentRegistry::register_component<Tag>("Tag");
    }
};

TEST_F(QueryTest, QueryWithSingleComponent) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();
    Entity e2 = world.create();

    world.add_component(e0, Position{1.0f, 2.0f});
    world.add_component(e1, Position{3.0f, 4.0f});
    world.add_component(e2, Velocity{5.0f, 6.0f});

    foundation::size_t count = 0;
    world.query<Position>().each([&](Entity, Position& pos) {
        ++count;
        EXPECT_NE(pos.x, 0.0f);
    });
    EXPECT_EQ(count, 2);
}

TEST_F(QueryTest, QueryWithMultipleComponents) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();
    Entity e2 = world.create();

    world.add_component(e0, Position{1.0f, 0.0f});
    world.add_component(e0, Velocity{0.1f, 0.0f});

    world.add_component(e1, Position{2.0f, 0.0f});

    world.add_component(e2, Position{3.0f, 0.0f});
    world.add_component(e2, Velocity{0.3f, 0.0f});

    foundation::size_t count = 0;
    world.query<Position, Velocity>().each([&](Entity, Position&, Velocity&) {
        ++count;
    });
    EXPECT_EQ(count, 2);
}

TEST_F(QueryTest, QueryWithoutFilter) {
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
    world.query<Position>().exclude<Tag>().each([&](Entity, Position&) {
        ++count;
    });
    EXPECT_EQ(count, 1);
}

TEST_F(QueryTest, QueryCount) {
    World world;
    Entity e0 = world.create();
    Entity e1 = world.create();
    world.add_component(e0, Position{});
    world.add_component(e1, Position{});

    EXPECT_EQ(world.query_count<Position>(), 2);
    EXPECT_EQ(world.query_count<Velocity>(), 0);
}

TEST_F(QueryTest, EachWithReadonlyAccess) {
    World world;
    Entity e0 = world.create();
    world.add_component(e0, Position{7.0f, 8.0f});

    world.query<Position>().each([](Entity, const Position& pos) {
        EXPECT_FLOAT_EQ(pos.x, 7.0f);
        EXPECT_FLOAT_EQ(pos.y, 8.0f);
    });
}

TEST_F(QueryTest, EachModifiesComponents) {
    World world;
    Entity e0 = world.create();
    world.add_component(e0, Position{0.0f, 0.0f});
    world.add_component(e0, Velocity{1.0f, 2.0f});

    world.query<Position, Velocity>().each([](Entity, Position& pos, const Velocity& vel) {
        pos.x += vel.dx;
        pos.y += vel.dy;
    });

    EXPECT_FLOAT_EQ(world.get_component<Position>(e0).x, 1.0f);
    EXPECT_FLOAT_EQ(world.get_component<Position>(e0).y, 2.0f);
}

} // namespace
