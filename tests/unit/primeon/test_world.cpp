#include <gtest/gtest.h>
#include <primeon/world/physics_world.hpp>

using namespace primeon::math;

// ═══════════════════════════════════════════════════════════════════════════
// PhysicsMaterial
// ═══════════════════════════════════════════════════════════════════════════

TEST(PhysicsMaterial, DefaultValues) {
    PhysicsMaterial mat;
    EXPECT_NEAR(mat.friction, 0.5f, kEpsilon);
    EXPECT_NEAR(mat.restitution, 0.0f, kEpsilon);
    EXPECT_NEAR(mat.density, 1.0f, kEpsilon);
}

TEST(PhysicsMaterial, CombineAverage) {
    f32 result = PhysicsMaterial::combineFriction(0.4f, 0.6f, MaterialCombine::Average);
    EXPECT_NEAR(result, 0.5f, kEpsilon);
}

TEST(PhysicsMaterial, CombineMin) {
    f32 result = PhysicsMaterial::combineFriction(0.4f, 0.6f, MaterialCombine::Min);
    EXPECT_NEAR(result, 0.4f, kEpsilon);
}

TEST(PhysicsMaterial, CombineMax) {
    f32 result = PhysicsMaterial::combineFriction(0.4f, 0.6f, MaterialCombine::Max);
    EXPECT_NEAR(result, 0.6f, kEpsilon);
}

TEST(PhysicsMaterial, CombineMultiply) {
    f32 result = PhysicsMaterial::combineRestitution(0.5f, 0.8f, MaterialCombine::Multiply);
    EXPECT_NEAR(result, 0.4f, kEpsilon);
}

TEST(PhysicsMaterial, CombinedFriction) {
    PhysicsMaterial a = PhysicsMaterial::rubber();
    PhysicsMaterial b = PhysicsMaterial::ice();
    f32 combined = a.combinedFriction(b);
    EXPECT_GT(combined, 0.0f);
    EXPECT_LT(combined, a.friction);  // average should be less than rubber
}

TEST(PhysicsMaterial, Presets) {
    EXPECT_NEAR(PhysicsMaterial::defaultMaterial().friction, 0.5f, kEpsilon);
    EXPECT_NEAR(PhysicsMaterial::frictionless().friction, 0.0f, kEpsilon);
    EXPECT_NEAR(PhysicsMaterial::bouncy().restitution, 0.8f, kEpsilon);
    EXPECT_NEAR(PhysicsMaterial::ice().friction, 0.05f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// RigidBody
// ═══════════════════════════════════════════════════════════════════════════

TEST(RigidBody, DefaultConstruction) {
    RigidBody body;
    EXPECT_EQ(body.id, kInvalidBody);
    EXPECT_EQ(body.type, BodyType::Dynamic);
    EXPECT_TRUE(body.isDynamic());
    EXPECT_FALSE(body.isStatic());
    EXPECT_FALSE(body.isKinematic());
}

TEST(RigidBody, TypeQueries) {
    RigidBody s;
    s.type = BodyType::Static;
    EXPECT_TRUE(s.isStatic());

    RigidBody k;
    k.type = BodyType::Kinematic;
    EXPECT_TRUE(k.isKinematic());
}

TEST(RigidBody, SleepState) {
    RigidBody body;
    body.type = BodyType::Dynamic;
    EXPECT_TRUE(body.isAwake());

    body.putToSleep();
    EXPECT_TRUE(body.isSleeping());
    EXPECT_NEAR(body.linearVelocity.length(), 0.0f, kEpsilon);

    body.wake();
    EXPECT_TRUE(body.isAwake());
}

TEST(RigidBody, ForceApplication) {
    RigidBody body;
    body.type = BodyType::Dynamic;
    body.sleepState = SleepState::Awake;
    body.massProperties = MassProperties::fromMass(1.0f);

    body.applyForce(Vector3(10, 0, 0));
    EXPECT_NEAR(body.pendingForces.force().x, 10.0f, kEpsilon);

    body.clearForces();
    EXPECT_TRUE(body.pendingForces.isZero());
}

TEST(RigidBody, ImpulseApplication) {
    RigidBody body;
    body.type = BodyType::Dynamic;
    body.sleepState = SleepState::Awake;
    body.massProperties = MassProperties::fromMass(2.0f);

    body.applyImpulse(Vector3(4, 0, 0));
    EXPECT_NEAR(body.linearVelocity.x, 2.0f, kEpsilon);  // 4 / 2 = 2
}

TEST(RigidBody, ImpulseAtPoint) {
    RigidBody body;
    body.type = BodyType::Dynamic;
    body.sleepState = SleepState::Awake;
    body.position = Vector3(0, 0, 0);
    body.massProperties = MassProperties::fromMass(1.0f);
    body.inverseInertia = InertiaTensor::solidBox(1.0f, Vector3(0.5f, 0.5f, 0.5f)).inverse();

    // Apply impulse at offset (1, 0, 0) — should produce both linear and angular velocity
    body.applyImpulse(Vector3(0, 10, 0), Vector3(1, 0, 0));
    EXPECT_GT(body.linearVelocity.y, 0.0f);
    EXPECT_GT(std::abs(body.angularVelocity.z), kEpsilon);
}

TEST(RigidBody, VelocityAtPoint) {
    RigidBody body;
    body.position = Vector3(0, 0, 0);
    body.linearVelocity = Vector3(1, 0, 0);
    body.angularVelocity = Vector3(0, 0, 1);  // spinning around Z

    Vector3 v = body.velocityAtPoint(Vector3(1, 0, 0));
    // v = linear + omega x r = (1,0,0) + (0,0,1)x(1,0,0) = (1,0,0) + (0,1,0) = (1,1,0)
    EXPECT_NEAR(v.x, 1.0f, kEpsilon);
    EXPECT_NEAR(v.y, 1.0f, kEpsilon);
}

TEST(RigidBody, DeadImpulseIgnored) {
    RigidBody body;
    body.type = BodyType::Dynamic;
    body.sleepState = SleepState::Sleeping;

    body.applyImpulse(Vector3(10, 0, 0));
    EXPECT_NEAR(body.linearVelocity.length(), 0.0f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// BodyManager
// ═══════════════════════════════════════════════════════════════════════════

TEST(BodyManager, CreateBody) {
    BodyManager mgr;
    BodyDescriptor desc = BodyDescriptor::dynamicBody(Vector3(1, 2, 3));
    BodyID id = mgr.createBody(desc);
    EXPECT_NE(id, kInvalidBody);
    EXPECT_TRUE(mgr.isValid(id));
    EXPECT_EQ(mgr.size(), 1u);
}

TEST(BodyManager, DestroyBody) {
    BodyManager mgr;
    BodyID id = mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    mgr.destroyBody(id);
    EXPECT_FALSE(mgr.isValid(id));
    EXPECT_EQ(mgr.size(), 0u);
}

TEST(BodyManager, ReuseSlots) {
    BodyManager mgr;
    BodyID id1 = mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    mgr.destroyBody(id1);
    BodyID id2 = mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    EXPECT_EQ(id1, id2);  // Reused slot
    EXPECT_TRUE(mgr.isValid(id2));
}

TEST(BodyManager, MaxBodies) {
    BodyManager mgr(4);
    for (u32 i = 0; i < 4; ++i) {
        EXPECT_NE(mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0))), kInvalidBody);
    }
    EXPECT_EQ(mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0))), kInvalidBody);
}

TEST(BodyManager, BodyTypes) {
    BodyManager mgr;
    BodyID s = mgr.createBody(BodyDescriptor::staticBody(Vector3(0, 0, 0)));
    BodyID d = mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0, 0, 0)));
    BodyID k = mgr.createBody(BodyDescriptor::kinematicBody(Vector3(0, 0, 0)));

    EXPECT_TRUE(mgr.getBody(s).isStatic());
    EXPECT_TRUE(mgr.getBody(d).isDynamic());
    EXPECT_TRUE(mgr.getBody(k).isKinematic());

    auto stats = mgr.getStats();
    EXPECT_EQ(stats.staticBodies, 1u);
    EXPECT_EQ(stats.dynamicBodies, 1u);
    EXPECT_EQ(stats.kinematicBodies, 1u);
}

TEST(BodyManager, ForceApplication) {
    BodyManager mgr;
    BodyID id = mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    mgr.applyForce(id, Vector3(5, 0, 0));
    EXPECT_NEAR(mgr.getBody(id).pendingForces.force().x, 5.0f, kEpsilon);
}

TEST(BodyManager, ImpulseApplication) {
    BodyManager mgr;
    BodyID id = mgr.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0), 1.0f));
    mgr.applyImpulse(id, Vector3(10, 0, 0));
    EXPECT_NEAR(mgr.getBody(id).linearVelocity.x, 10.0f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Island Generation
// ═══════════════════════════════════════════════════════════════════════════

TEST(UnionFind, Basics) {
    UnionFind uf(5);
    EXPECT_TRUE(uf.connected(0, 0));
    EXPECT_FALSE(uf.connected(0, 1));

    uf.unite(0, 1);
    EXPECT_TRUE(uf.connected(0, 1));

    uf.unite(2, 3);
    EXPECT_TRUE(uf.connected(2, 3));
    EXPECT_FALSE(uf.connected(0, 2));

    uf.unite(1, 3);
    EXPECT_TRUE(uf.connected(0, 3));
    EXPECT_TRUE(uf.connected(1, 2));
}

TEST(IslandBuilder, SingleIsland) {
    // 3 bodies: 0-1 connected, 1-2 connected → one island
    RigidBody bodies[3];
    for (u32 i = 0; i < 3; ++i) {
        bodies[i].id = i;
        bodies[i].enabled = true;
        bodies[i].type = BodyType::Dynamic;
    }

    std::pair<u32, u32> pairs[] = {{0, 1}, {1, 2}};

    IslandBuilder builder(3);
    const auto& islands = builder.build(3, pairs, 2, bodies);
    EXPECT_EQ(islands.size(), 1u);
    EXPECT_EQ(islands[0].bodies.size(), 3u);
    EXPECT_EQ(islands[0].contacts.size(), 2u);
}

TEST(IslandBuilder, TwoIslands) {
    // 4 bodies: {0,1} and {2,3} — separate islands
    RigidBody bodies[4];
    for (u32 i = 0; i < 4; ++i) {
        bodies[i].id = i;
        bodies[i].enabled = true;
        bodies[i].type = BodyType::Dynamic;
    }

    std::pair<u32, u32> pairs[] = {{0, 1}, {2, 3}};

    IslandBuilder builder(4);
    const auto& islands = builder.build(4, pairs, 2, bodies);
    EXPECT_EQ(islands.size(), 2u);

    // Each island should have 2 bodies and 1 contact
    for (const auto& island : islands) {
        EXPECT_EQ(island.bodies.size(), 2u);
        EXPECT_EQ(island.contacts.size(), 1u);
    }
}

TEST(IslandBuilder, NoContacts) {
    RigidBody bodies[3];
    for (u32 i = 0; i < 3; ++i) {
        bodies[i].id = i;
        bodies[i].enabled = true;
    }

    IslandBuilder builder(3);
    const auto& islands = builder.build(3, nullptr, 0, bodies);
    // 3 separate islands
    EXPECT_EQ(islands.size(), 3u);
}

TEST(IslandBuilder, StaticBodies) {
    RigidBody bodies[3];
    bodies[0].id = 0; bodies[0].type = BodyType::Static; bodies[0].enabled = true;
    bodies[1].id = 1; bodies[1].type = BodyType::Dynamic; bodies[1].enabled = true;
    bodies[2].id = 2; bodies[2].type = BodyType::Dynamic; bodies[2].enabled = true;

    std::pair<u32, u32> pairs[] = {{1, 2}};

    IslandBuilder builder(3);
    const auto& islands = builder.build(3, pairs, 1, bodies);
    // Static body 0 gets its own island, 1+2 form another
    EXPECT_GE(islands.size(), 2u);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sleeping
// ═══════════════════════════════════════════════════════════════════════════

TEST(SleepSystem, AtRest) {
    SleepSystem sys;
    RigidBody body;
    body.type = BodyType::Dynamic;
    body.linearVelocity = Vector3(0, 0, 0);
    body.angularVelocity = Vector3(0, 0, 0);
    body.enabled = true;

    EXPECT_TRUE(sys.isBodyAtRest(body));
}

TEST(SleepSystem, Moving) {
    SleepSystem sys;
    RigidBody body;
    body.type = BodyType::Dynamic;
    body.linearVelocity = Vector3(1, 0, 0);
    body.enabled = true;

    EXPECT_FALSE(sys.isBodyAtRest(body));
}

TEST(SleepSystem, StaticNeverSleeps) {
    SleepSystem sys;
    RigidBody body;
    body.type = BodyType::Static;
    body.enabled = true;

    EXPECT_FALSE(sys.isBodyAtRest(body));
}

TEST(SleepSystem, SleepAfterThreshold) {
    SleepConfig cfg;
    cfg.sleepTimeThreshold = 3;
    SleepSystem sys;
    sys.config = cfg;

    std::vector<RigidBody> bodies(1);
    bodies[0].id = 0;
    bodies[0].type = BodyType::Dynamic;
    bodies[0].sleepState = SleepState::Awake;
    bodies[0].enabled = true;
    bodies[0].allowSleep = true;
    bodies[0].linearVelocity = Vector3(0, 0, 0);
    bodies[0].angularVelocity = Vector3(0, 0, 0);

    // Run 2 frames — should not sleep yet
    sys.update(bodies);
    sys.update(bodies);
    EXPECT_FALSE(bodies[0].isSleeping());

    // Run 1 more frame — should sleep
    sys.update(bodies);
    EXPECT_TRUE(bodies[0].isSleeping());
}

TEST(SleepSystem, WakeOnMovement) {
    SleepConfig cfg;
    cfg.sleepTimeThreshold = 2;
    SleepSystem sys;
    sys.config = cfg;

    std::vector<RigidBody> bodies(1);
    bodies[0].id = 0;
    bodies[0].type = BodyType::Dynamic;
    bodies[0].sleepState = SleepState::Awake;
    bodies[0].enabled = true;
    bodies[0].allowSleep = true;

    // Sleep
    bodies[0].linearVelocity = Vector3(0, 0, 0);
    sys.update(bodies);
    sys.update(bodies);
    sys.update(bodies);
    EXPECT_TRUE(bodies[0].isSleeping());

    // Wake by moving
    bodies[0].linearVelocity = Vector3(5, 0, 0);
    sys.update(bodies);
    EXPECT_TRUE(bodies[0].isAwake());
}

TEST(SleepSystem, AllowSleepFalse) {
    SleepSystem sys;
    std::vector<RigidBody> bodies(1);
    bodies[0].id = 0;
    bodies[0].type = BodyType::Dynamic;
    bodies[0].sleepState = SleepState::Awake;
    bodies[0].enabled = true;
    bodies[0].allowSleep = false;
    bodies[0].linearVelocity = Vector3(0, 0, 0);

    for (int i = 0; i < 100; ++i) sys.update(bodies);
    EXPECT_TRUE(bodies[0].isAwake());
}

// ═══════════════════════════════════════════════════════════════════════════
// Callbacks
// ═══════════════════════════════════════════════════════════════════════════

static bool g_bodyCreatedCalled = false;
static bool g_bodyDestroyedCalled = false;
static u32 g_lastBodyID = kInvalidBody;

static void onBodyCreated(u32 id, void*) {
    g_bodyCreatedCalled = true;
    g_lastBodyID = id;
}

static void onBodyDestroyed(u32 id, void*) {
    g_bodyDestroyedCalled = true;
    g_lastBodyID = id;
}

TEST(Callbacks, BodyCreated) {
    g_bodyCreatedCalled = false;
    g_lastBodyID = kInvalidBody;

    WorldConfig cfg;
    PhysicsWorld world(cfg);
    world.callbacks.onBodyCreated = onBodyCreated;

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    EXPECT_TRUE(g_bodyCreatedCalled);
    EXPECT_EQ(g_lastBodyID, id);
}

TEST(Callbacks, BodyDestroyed) {
    g_bodyDestroyedCalled = false;
    g_lastBodyID = kInvalidBody;

    WorldConfig cfg;
    PhysicsWorld world(cfg);
    world.callbacks.onBodyDestroyed = onBodyDestroyed;

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    world.destroyBody(id);
    EXPECT_TRUE(g_bodyDestroyedCalled);
    EXPECT_EQ(g_lastBodyID, id);
}

// ═══════════════════════════════════════════════════════════════════════════
// PhysicsWorld
// ═══════════════════════════════════════════════════════════════════════════

TEST(PhysicsWorld, CreateDestroy) {
    WorldConfig cfg;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 10, 0)));
    EXPECT_TRUE(world.isValidBody(id));
    EXPECT_EQ(world.getBodyCount(), 1u);

    world.destroyBody(id);
    EXPECT_FALSE(world.isValidBody(id));
    EXPECT_EQ(world.getBodyCount(), 0u);
}

TEST(PhysicsWorld, StepEmpty) {
    WorldConfig cfg;
    PhysicsWorld world(cfg);
    world.step(1.0f / 60.0f);  // No bodies — should not crash
    EXPECT_EQ(world.getFrameCount(), 1u);
}

TEST(PhysicsWorld, GravityFalls) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 100, 0), 1.0f));
    world.step(1.0f / 60.0f);

    const RigidBody& body = world.getBody(id);
    // After one step, body should have downward velocity (gravity)
    EXPECT_LT(body.linearVelocity.y, 0.0f);
    // Body should have moved down slightly
    EXPECT_LT(body.position.y, 100.0f);
}

TEST(PhysicsWorld, StaticBodyImmobile) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::staticBody(Vector3(0, 100, 0)));

    Vector3 posBefore = world.getBody(id).position;
    world.step(1.0f / 60.0f);

    EXPECT_NEAR(world.getBody(id).position.x, posBefore.x, kEpsilon);
    EXPECT_NEAR(world.getBody(id).position.y, posBefore.y, kEpsilon);
    EXPECT_NEAR(world.getBody(id).position.z, posBefore.z, kEpsilon);
}

TEST(PhysicsWorld, KinematicBodyMovesByVelocity) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::kinematicBody(Vector3(0, 0, 0)));
    world.getBody(id).linearVelocity = Vector3(1, 0, 0);

    world.step(1.0f / 60.0f);

    // Kinematic body should not be affected by gravity
    EXPECT_NEAR(world.getBody(id).linearVelocity.y, 0.0f, kEpsilon);
}

TEST(PhysicsWorld, SleepingWorks) {
    WorldConfig cfg;
    cfg.sleepingEnabled = true;
    cfg.sleep.sleepTimeThreshold = 3;
    cfg.sleep.linearSleepThreshold = 0.1f;
    cfg.gravity = Vector3(0, 0, 0);  // No gravity — body stays at rest
    PhysicsWorld world(cfg);

    // Create a body at rest
    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 0, 0), 1.0f));

    // Run several steps — body should go to sleep
    for (int i = 0; i < 10; ++i) {
        world.step(1.0f / 60.0f);
    }

    EXPECT_TRUE(world.getBody(id).isSleeping());
}

TEST(PhysicsWorld, WakeAll) {
    WorldConfig cfg;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0,0,0)));
    world.sleepBody(id);
    EXPECT_TRUE(world.getBody(id).isSleeping());

    world.wakeAll();
    EXPECT_TRUE(world.getBody(id).isAwake());
}

TEST(PhysicsWorld, ApplyForce) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 0, 0), 1.0f));
    world.applyForce(id, Vector3(100, 0, 0));

    world.step(1.0f / 60.0f);

    // Body should have acquired horizontal velocity
    EXPECT_GT(world.getBody(id).linearVelocity.x, 0.0f);
}

TEST(PhysicsWorld, ApplyImpulse) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 0, 0), 1.0f));
    world.applyImpulse(id, Vector3(10, 0, 0));

    // Immediate velocity change
    EXPECT_NEAR(world.getBody(id).linearVelocity.x, 10.0f, kEpsilon);
}

TEST(PhysicsWorld, Statistics) {
    WorldConfig cfg;
    PhysicsWorld world(cfg);

    BodyID b1 = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 0, 0)));
    BodyID b2 = world.createBody(BodyDescriptor::staticBody(Vector3(5, 0, 0)));
    (void)b1; (void)b2;

    world.step(1.0f / 60.0f);

    auto stats = world.getStats();
    EXPECT_EQ(stats.totalBodies, 2u);
    EXPECT_EQ(stats.dynamicBodies, 1u);
    EXPECT_EQ(stats.staticBodies, 1u);
}

TEST(PhysicsWorld, MultipleBodies) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    cfg.gravity = Vector3(0, 0, 0);
    cfg.maxBodies = 16;
    PhysicsWorld world(cfg);

    BodyID b0 = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 0, 0), 1.0f));
    BodyID b1 = world.createBody(BodyDescriptor::dynamicBody(Vector3(100, 0, 0), 1.0f));

    world.step(1.0f / 60.0f);
    world.step(1.0f / 60.0f);
    world.step(1.0f / 60.0f);

    EXPECT_EQ(world.getFrameCount(), 3u);
    EXPECT_EQ(world.getBodyCount(), 2u);
    EXPECT_TRUE(world.isValidBody(b0));
    EXPECT_TRUE(world.isValidBody(b1));
}

TEST(PhysicsWorld, FixedTimestep) {
    WorldConfig cfg;
    cfg.fixedDt = 1.0f / 60.0f;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 100, 0)));
    (void)id;

    // Step with frame time less than fixed dt — should not simulate
    world.step(1.0f / 120.0f);
    EXPECT_EQ(world.getFrameCount(), 0u);

    // Step with exactly fixed dt
    world.step(1.0f / 60.0f);
    EXPECT_EQ(world.getFrameCount(), 1u);
}

TEST(PhysicsWorld, StepFixed) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 100, 0)));
    world.stepFixed(1.0f / 60.0f);

    EXPECT_EQ(world.getFrameCount(), 1u);
    EXPECT_LT(world.getBody(id).linearVelocity.y, 0.0f);
}

TEST(PhysicsWorld, ConfigPresets) {
    auto gameCfg = WorldConfig::gameDefault();
    EXPECT_EQ(gameCfg.solver.velocityIterations, 8u);

    auto simCfg = WorldConfig::simulation();
    EXPECT_EQ(simCfg.solver.velocityIterations, 20u);

    auto fastCfg = WorldConfig::fast();
    EXPECT_EQ(fastCfg.solver.velocityIterations, 4u);
}

TEST(PhysicsWorld, EnableDisable) {
    WorldConfig cfg;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 100, 0)));
    world.disableBody(id);
    EXPECT_FALSE(world.getBody(id).enabled);

    world.enableBody(id);
    EXPECT_TRUE(world.getBody(id).enabled);
}

// ═══════════════════════════════════════════════════════════════════════════
// Simulation Stats
// ═══════════════════════════════════════════════════════════════════════════

TEST(SimulationStats, DefaultValues) {
    SimulationStats stats;
    EXPECT_EQ(stats.totalBodies, 0u);
    EXPECT_EQ(stats.contactCount, 0u);
}
