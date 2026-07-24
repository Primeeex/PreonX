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

class SystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        ComponentRegistry::register_component<Position>("Position");
        ComponentRegistry::register_component<Velocity>("Velocity");
    }
};

TEST_F(SystemTest, AddAndRunSingleSystem) {
    World world;
    Entity e0 = world.create();
    world.add_component(e0, Position{0.0f, 0.0f});
    world.add_component(e0, Velocity{1.0f, 2.0f});

    bool ran = false;
    world.add_system([&](World& w) {
        ran = true;
        w.query<Position, Velocity>().each([](Entity, Position& pos, const Velocity& vel) {
            pos.x += vel.dx;
            pos.y += vel.dy;
        });
    });

    world.run_systems();
    EXPECT_TRUE(ran);
    EXPECT_FLOAT_EQ(world.get_component<Position>(e0).x, 1.0f);
    EXPECT_FLOAT_EQ(world.get_component<Position>(e0).y, 2.0f);
}

TEST_F(SystemTest, MultipleSystemsRunInOrder) {
    World world;
    Entity e0 = world.create();
    world.add_component(e0, Position{0.0f, 0.0f});

    int order = 0;
    int system1_order = 0;
    int system2_order = 0;

    world.add_system([&](World&) {
        system1_order = ++order;
    });
    world.add_system([&](World&) {
        system2_order = ++order;
    });

    world.run_systems();
    EXPECT_EQ(system1_order, 1);
    EXPECT_EQ(system2_order, 2);
}

TEST_F(SystemTest, NoSystemsIsNoop) {
    World world;
    world.run_systems();
}

} // namespace
