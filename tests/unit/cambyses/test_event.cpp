#include <gtest/gtest.h>

#include "cambyses/events/event_dispatcher.hpp"

using namespace cambyses;

namespace {

struct CollisionEvent {
    Entity a;
    Entity b;
    float force;
};

struct DamageEvent {
    Entity target;
    int amount;
};

class EventDispatcherTest : public ::testing::Test {
protected:
    EventDispatcher dispatcher;
};

TEST_F(EventDispatcherTest, PublishWithNoSubscribers) {
    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 5.0f});
}

TEST_F(EventDispatcherTest, SubscribeAndPublish) {
    int call_count = 0;
    float received_force = 0.0f;

    dispatcher.subscribe<CollisionEvent>([&](const CollisionEvent& e) {
        ++call_count;
        received_force = e.force;
    });

    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 42.0f});
    EXPECT_EQ(call_count, 1);
    EXPECT_FLOAT_EQ(received_force, 42.0f);
}

TEST_F(EventDispatcherTest, MultipleSubscribers) {
    int count_a = 0;
    int count_b = 0;

    dispatcher.subscribe<CollisionEvent>([&](const CollisionEvent&) { ++count_a; });
    dispatcher.subscribe<CollisionEvent>([&](const CollisionEvent&) { ++count_b; });

    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 1.0f});
    EXPECT_EQ(count_a, 1);
    EXPECT_EQ(count_b, 1);
}

TEST_F(EventDispatcherTest, DifferentEventTypes) {
    int collision_count = 0;
    int damage_count = 0;

    dispatcher.subscribe<CollisionEvent>([&](const CollisionEvent&) { ++collision_count; });
    dispatcher.subscribe<DamageEvent>([&](const DamageEvent&) { ++damage_count; });

    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 1.0f});
    dispatcher.publish(DamageEvent{{2, 0}, 10});

    EXPECT_EQ(collision_count, 1);
    EXPECT_EQ(damage_count, 1);
}

TEST_F(EventDispatcherTest, Unsubscribe) {
    int count = 0;
    auto id = dispatcher.subscribe<CollisionEvent>([&](const CollisionEvent&) { ++count; });

    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 1.0f});
    EXPECT_EQ(count, 1);

    dispatcher.unsubscribe<CollisionEvent>(id);
    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 1.0f});
    EXPECT_EQ(count, 1);
}

TEST_F(EventDispatcherTest, ClearRemovesAll) {
    int count = 0;
    dispatcher.subscribe<CollisionEvent>([&](const CollisionEvent&) { ++count; });

    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 1.0f});
    EXPECT_EQ(count, 1);

    dispatcher.clear();
    dispatcher.publish(CollisionEvent{{0, 0}, {1, 0}, 1.0f});
    EXPECT_EQ(count, 1);
}

TEST_F(EventDispatcherTest, PublishMultipleTimes) {
    int count = 0;
    dispatcher.subscribe<DamageEvent>([&](const DamageEvent&) { ++count; });

    for (int i = 0; i < 10; ++i) {
        dispatcher.publish(DamageEvent{{0, 0}, i});
    }
    EXPECT_EQ(count, 10);
}

} // namespace
