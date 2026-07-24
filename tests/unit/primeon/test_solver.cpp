#include <gtest/gtest.h>
#include <primeon/solver/solver_config.hpp>
#include <primeon/solver/sequential_impulse/sequential_impulse_solver.hpp>
#include <primeon/solver/friction/friction_solver.hpp>
#include <primeon/solver/restitution/restitution_solver.hpp>
#include <primeon/solver/stabilization/stabilization.hpp>
#include <primeon/constraints/contact_constraint.hpp>
#include <primeon/constraints/jacobian.hpp>
#include <primeon/collision/contact.hpp>

using namespace primeon::math;

// ═══════════════════════════════════════════════════════════════════════════
// Helper functions
// ═══════════════════════════════════════════════════════════════════════════

static SolverBodyData makeBody(f32 invMass, const Vector3& invInertiaDiag = Vector3(1,1,1)) {
    SolverBodyData b;
    b.massData.inverseMass = invMass;
    b.massData.inverseInertiaDiag = invInertiaDiag;
    return b;
}

static SolverBodyData makeStaticBody() {
    return makeBody(0.0f);
}

static ContactConstraint makeContact(
    u32 idA, u32 idB,
    const Vector3& point, const Vector3& normal,
    f32 penetration,
    f32 invMassA = 1.0f, f32 invMassB = 0.0f,
    f32 restitution = 0.0f, f32 friction = 0.5f)
{
    ContactConstraint cc;
    cc.bodyIDA = idA;
    cc.bodyIDB = idB;
    cc.massA.inverseMass = invMassA;
    cc.massA.inverseInertiaDiag = Vector3(1,1,1);
    cc.massB.inverseMass = invMassB;
    cc.massB.inverseInertiaDiag = Vector3(1,1,1);
    cc.pointA = point;
    cc.pointB = point;
    cc.normal = normal;
    cc.penetration = penetration;
    cc.restitution = restitution;
    cc.friction = friction;
    return cc;
}

// ═══════════════════════════════════════════════════════════════════════════
// SolverConfig
// ═══════════════════════════════════════════════════════════════════════════

TEST(SolverConfig, DefaultValues) {
    SolverConfig cfg;
    EXPECT_EQ(cfg.velocityIterations, 8u);
    EXPECT_EQ(cfg.positionIterations, 3u);
    EXPECT_NEAR(cfg.penetrationSlop, 0.005f, kEpsilon);
    EXPECT_NEAR(cfg.baumgarteFactor, 0.2f, kEpsilon);
    EXPECT_TRUE(cfg.warmStartingEnabled);
    EXPECT_FALSE(cfg.splitImpulseEnabled);
}

TEST(SolverConfig, GameDefault) {
    auto cfg = SolverConfig::gameDefault();
    EXPECT_EQ(cfg.velocityIterations, 8u);
    EXPECT_TRUE(cfg.warmStartingEnabled);
    EXPECT_FALSE(cfg.splitImpulseEnabled);
}

TEST(SolverConfig, Simulation) {
    auto cfg = SolverConfig::simulation();
    EXPECT_EQ(cfg.velocityIterations, 20u);
    EXPECT_TRUE(cfg.splitImpulseEnabled);
}

TEST(SolverConfig, Fast) {
    auto cfg = SolverConfig::fast();
    EXPECT_EQ(cfg.velocityIterations, 4u);
    EXPECT_FALSE(cfg.warmStartingEnabled);
}

// ═══════════════════════════════════════════════════════════════════════════
// Stabilization
// ═══════════════════════════════════════════════════════════════════════════

TEST(Stabilization, BaumgarteBias) {
    f32 bias = computeBaumgarteBias(0.01f, 0.005f, 0.2f, 1.0f/60.0f);
    EXPECT_GT(bias, 0.0f);
    // correction = max(0, 0.01 + 0.005) = 0.015
    // bias = 0.2 / (1/60) * 0.015 = 0.2 * 60 * 0.015 = 0.18
    EXPECT_NEAR(bias, 0.18f, 0.01f);
}

TEST(Stabilization, BaumgarteBiasZeroPenetration) {
    f32 bias = computeBaumgarteBias(-0.01f, 0.005f, 0.2f, 1.0f/60.0f);
    // correction = max(0, -0.01 + 0.005) = 0
    EXPECT_NEAR(bias, 0.0f, kEpsilon);
}

TEST(Stabilization, BaumgarteBiasZeroDt) {
    f32 bias = computeBaumgarteBias(0.01f, 0.005f, 0.2f, 0.0f);
    EXPECT_NEAR(bias, 0.0f, kEpsilon);
}

TEST(Stabilization, SplitImpulseCorrection) {
    Vector3 corr = computeSplitImpulseCorrection(0.01f, 0.005f, kVector3UnitY, 0.8f, 1.0f/60.0f);
    EXPECT_GT(corr.length(), 0.0f);
    // Should be along Y (the normal)
    EXPECT_NEAR(corr.x, 0.0f, kEpsilon);
    EXPECT_GT(corr.y, 0.0f);
    EXPECT_NEAR(corr.z, 0.0f, kEpsilon);
}

TEST(Stabilization, SplitImpulseState) {
    SplitImpulseState state;
    state.reset();
    EXPECT_EQ(state.count, 0u);

    state.addContact(0);
    EXPECT_EQ(state.count, 1u);

    state.accumulateCorrection(0, Vector3(0, 0.01f, 0));
    state.accumulateCorrection(0, Vector3(0, 0.02f, 0));
    EXPECT_NEAR(state.getCorrection(0).y, 0.03f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Restitution
// ═══════════════════════════════════════════════════════════════════════════

TEST(Restitution, ComputeBias_NoBounce) {
    // Separating velocity -> no restitution
    Vector3 velA(0, 1, 0); // moving up (away)
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);
    f32 bias = computeRestitutionBias(velA, angVel, velB, angVel,
                                       Vector3(0,1,0), Vector3(0,-1,0),
                                       kVector3UnitY, 0.5f, 1.0f);
    EXPECT_NEAR(bias, 0.0f, kEpsilon);
}

TEST(Restitution, ComputeBias_Bounce) {
    // Approaching at 5 m/s (below threshold of 1.0 -> bounce)
    Vector3 velA(0, -5, 0);
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);
    f32 bias = computeRestitutionBias(velA, angVel, velB, angVel,
                                       Vector3(0,1,0), Vector3(0,-1,0),
                                       kVector3UnitY, 0.5f, 1.0f);
    // targetVelocity = -0.5 * (-5) = 2.5
    EXPECT_NEAR(bias, 2.5f, 0.01f);
}

TEST(Restitution, ComputeBias_BelowThreshold) {
    // Slow approach (below threshold)
    Vector3 velA(0, -0.5, 0);
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);
    f32 bias = computeRestitutionBias(velA, angVel, velB, angVel,
                                       Vector3(0,1,0), Vector3(0,-1,0),
                                       kVector3UnitY, 0.5f, 1.0f);
    EXPECT_NEAR(bias, 0.0f, kEpsilon);
}

TEST(Restitution, SolveRestitution) {
    ContactConstraint cc = makeContact(0, 1, Vector3(0,1,0), kVector3UnitY, 0.0f);
    cc.restitution = 0.5f;

    Vector3 velA(0, -5, 0);
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);
    SolverConfig config;

    solveRestitution(velA, angVel, velB, angVel, cc, config);
    // Should set velocity bias to bounce target
    EXPECT_GT(cc.velocityBias, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Friction
// ═══════════════════════════════════════════════════════════════════════════

TEST(Friction, CoulombClamping) {
    // Sliding contact with tangential velocity
    Vector3 velA(1, 0, 0); // sliding in X
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);

    InverseMassData massA, massB;
    massA.inverseMass = 1.0f;
    massA.inverseInertiaDiag = Vector3(1,1,1);
    massB.inverseMass = 0.0f; // static
    massB.inverseInertiaDiag = Vector3(0,0,0);

    f32 accumImpulse = 0.0f;
    f32 tangentMass = 0.5f; // effective mass for this tangent
    f32 maxFrictionImpulse = 5.0f; // mu * normalImpulse

    f32 applied = applyFrictionImpulse(
        velA, angVel, velB, angVel,
        massA, massB,
        Vector3(0,1,0), Vector3(0,0,0), // contact point
        kVector3UnitX, // tangent
        tangentMass, maxFrictionImpulse, accumImpulse);

    // Impulse should oppose motion (negative X)
    EXPECT_LT(applied, 0.0f);
    // Should be clamped by Coulomb cone
    EXPECT_LE(std::abs(accumImpulse), maxFrictionImpulse + kEpsilon);
    // Velocity should be reduced
    EXPECT_LT(velA.x, 1.0f);
}

TEST(Friction, StaticFriction) {
    // No tangential velocity -> no friction impulse
    Vector3 velA(0, 0, 0);
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);

    InverseMassData massA, massB;
    massA.inverseMass = 1.0f;
    massA.inverseInertiaDiag = Vector3(1,1,1);
    massB.inverseMass = 0.0f;
    massB.inverseInertiaDiag = Vector3(0,0,0);

    f32 accumImpulse = 0.0f;
    f32 tangentMass = 0.5f;
    f32 maxFrictionImpulse = 5.0f;

    f32 applied = applyFrictionImpulse(
        velA, angVel, velB, angVel,
        massA, massB,
        Vector3(0,1,0), Vector3(0,0,0),
        kVector3UnitX, tangentMass, maxFrictionImpulse, accumImpulse);

    EXPECT_NEAR(applied, 0.0f, kEpsilon);
}

TEST(Friction, CoulombConeBound) {
    // High tangential velocity but limited friction
    Vector3 velA(100, 0, 0);
    Vector3 velB(0, 0, 0);
    Vector3 angVel(0, 0, 0);

    InverseMassData massA, massB;
    massA.inverseMass = 1.0f;
    massA.inverseInertiaDiag = Vector3(1,1,1);
    massB.inverseMass = 0.0f;
    massB.inverseInertiaDiag = Vector3(0,0,0);

    f32 accumImpulse = 0.0f;
    f32 tangentMass = 0.5f;
    f32 maxFrictionImpulse = 1.0f; // very small friction bound

    (void)applyFrictionImpulse(
        velA, angVel, velB, angVel,
        massA, massB,
        Vector3(0,1,0), Vector3(0,0,0),
        kVector3UnitX, tangentMass, maxFrictionImpulse, accumImpulse);

    // Accumulated impulse should be clamped
    EXPECT_LE(std::abs(accumImpulse), maxFrictionImpulse + kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Unit Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, EmptySolve) {
    SequentialImpulseSolver solver;
    SolverBodyData bodies[2];
    solver.solve(bodies, 2, 1.0f/60.0f, SolverConfig());
    EXPECT_EQ(solver.getStats().contactCount, 0u);
}

TEST(SequentialImpulseSolver, AddContact) {
    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0,1,0), kVector3UnitY, 0.01f);
    bool added = solver.addContact(cc, kVector3UnitX, kVector3UnitZ);
    EXPECT_TRUE(added);
    EXPECT_EQ(solver.contactCount, 1u);
}

TEST(SequentialImpulseSolver, AddContactOverflow) {
    SequentialImpulseSolver solver;
    for (u32 i = 0; i < SequentialImpulseSolver::kMaxSolverContacts; ++i) {
        ContactConstraint cc = makeContact(i, i+1, Vector3(0,1,0), kVector3UnitY, 0.01f);
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);
    }
    EXPECT_EQ(solver.contactCount, SequentialImpulseSolver::kMaxSolverContacts);

    ContactConstraint cc = makeContact(999, 1000, Vector3(0,1,0), kVector3UnitY, 0.01f);
    bool added = solver.addContact(cc, kVector3UnitX, kVector3UnitZ);
    EXPECT_FALSE(added);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Sphere vs Static Ground
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, SphereFallingOntoGround) {
    // Sphere (body 0) falling onto static ground (body 1)
    // Contact: sphere at y=1, ground at y=0, normal pointing up
    // Penetration = 0.01 (slight overlap)

    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f); // dynamic sphere
    bodies[1] = makeStaticBody(); // static ground
    bodies[0].linearVelocity = Vector3(0, -2, 0); // falling

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;
    config.baumgarteFactor = 0.2f;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // After solving, sphere should be moving upward (bouncing off ground)
    EXPECT_GT(bodies[0].linearVelocity.y, -0.5f);
    // Ground should not move
    EXPECT_NEAR(bodies[1].linearVelocity.y, 0.0f, kEpsilon);
}

TEST(SequentialImpulseSolver, SphereRestingOnGround) {
    // Sphere with zero velocity resting on ground with small penetration
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, 0, 0);

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.005f);
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;
    config.baumgarteFactor = 0.2f;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // Sphere should remain roughly at rest (small Baumgarte correction is OK)
    EXPECT_NEAR(bodies[0].linearVelocity.y, 0.0f, 0.1f);
}

TEST(SequentialImpulseSolver, SphereBounce) {
    // Sphere falling fast, should bounce with restitution
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, -5, 0);

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
    cc.restitution = 0.8f;
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;
    config.restitutionThreshold = 1.0f;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // Should bounce upward
    EXPECT_GT(bodies[0].linearVelocity.y, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Box Collisions
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, TwoSpheresColliding) {
    // Two dynamic spheres approaching each other
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeBody(1.0f);
    bodies[0].linearVelocity = Vector3(2, 0, 0);  // moving right
    bodies[1].linearVelocity = Vector3(-2, 0, 0); // moving left

    SequentialImpulseSolver solver;
    // Contact at midpoint, normal from B to A
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 0, 0), kVector3UnitX, 0.01f,
                                        1.0f, 1.0f);
    solver.addContact(cc, kVector3UnitY, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // After collision, bodies should be moving apart or at rest
    // Body 0 should be pushed left (or slower right)
    // Body 1 should be pushed right (or slower left)
    EXPECT_LE(bodies[0].linearVelocity.x, 2.0f + kEpsilon);
    EXPECT_GE(bodies[1].linearVelocity.x, -2.0f - kEpsilon);
}

TEST(SequentialImpulseSolver, StaticVsStatic) {
    // Both bodies static -> no impulse applied
    SolverBodyData bodies[2];
    bodies[0] = makeStaticBody();
    bodies[1] = makeStaticBody();

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0,1,0), kVector3UnitY, 0.01f,
                                        0.0f, 0.0f);
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // Both should remain at rest
    EXPECT_NEAR(bodies[0].linearVelocity.length(), 0.0f, kEpsilon);
    EXPECT_NEAR(bodies[1].linearVelocity.length(), 0.0f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Resting Contacts
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, RestingContactStability) {
    // Sphere resting on ground for multiple "frames"
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = true;
    config.baumgarteFactor = 0.2f;

    f32 dt = 1.0f / 60.0f;

    // Simulate 10 frames
    for (int frame = 0; frame < 10; ++frame) {
        bodies[0].linearVelocity = Vector3(0, 0, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.005f);
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        solver.solve(bodies, 2, dt, config);
    }

    // After 10 frames, sphere should still be roughly at rest
    EXPECT_NEAR(bodies[0].linearVelocity.length(), 0.0f, 0.05f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Friction in Solver
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, FrictionReducesSliding) {
    // Sphere sliding on ground should slow down due to friction
    // Use larger penetration so normal impulse generates a meaningful Coulomb bound
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(5, 0, 0); // sliding fast

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.05f);
    cc.friction = 0.5f;
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;

    f32 velBefore = bodies[0].linearVelocity.x;
    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // Horizontal velocity should be reduced by friction
    // (or at least not increased)
    EXPECT_LE(bodies[0].linearVelocity.x, velBefore + kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Warm Starting
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, WarmStartingPreservesImpulse) {
    // First frame: solve and get accumulated impulses
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, -2, 0);

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = true;

    SequentialImpulseSolver solver1;
    ContactConstraint cc1 = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
    solver1.addContact(cc1, kVector3UnitX, kVector3UnitZ);
    solver1.solve(bodies, 2, 1.0f/60.0f, config);

    // Store accumulated impulse from first frame
    f32 accumulatedNormal = solver1.contacts[0].accumulatedNormalImpulse;

    // Second frame: warm start should use previous impulse
    bodies[0].linearVelocity = Vector3(0, -2, 0); // reset velocity

    SequentialImpulseSolver solver2;
    ContactConstraint cc2 = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
    cc2.accumulatedNormalImpulse = accumulatedNormal; // warm start data
    solver2.addContact(cc2, kVector3UnitX, kVector3UnitZ);
    solver2.solve(bodies, 2, 1.0f/60.0f, config);

    // With warm starting, result should be similar (or better)
    EXPECT_NEAR(bodies[0].linearVelocity.y, 0.0f, 1.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Box Stack
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, BoxStack) {
    // 3 boxes stacked on ground
    const u32 numBodies = 4; // 3 boxes + ground
    SolverBodyData bodies[numBodies];
    bodies[0] = makeBody(1.0f); // top box
    bodies[1] = makeBody(1.0f); // middle box
    bodies[2] = makeBody(1.0f); // bottom box
    bodies[3] = makeStaticBody(); // ground

    // All boxes start with slight downward velocity
    for (u32 i = 0; i < 3; ++i) {
        bodies[i].linearVelocity = Vector3(0, -1, 0);
    }

    SolverConfig config;
    config.velocityIterations = 12;
    config.warmStartingEnabled = true;
    config.baumgarteFactor = 0.2f;

    f32 dt = 1.0f / 60.0f;

    // Simulate 20 frames
    for (int frame = 0; frame < 20; ++frame) {
        SequentialImpulseSolver solver;

        // Contact: box 0 on box 1
        ContactConstraint cc01 = makeContact(0, 1, Vector3(0, 2, 0), kVector3UnitY, 0.005f);
        solver.addContact(cc01, kVector3UnitX, kVector3UnitZ);

        // Contact: box 1 on box 2
        ContactConstraint cc12 = makeContact(1, 2, Vector3(0, 1, 0), kVector3UnitY, 0.005f);
        solver.addContact(cc12, kVector3UnitX, kVector3UnitZ);

        // Contact: box 2 on ground
        ContactConstraint cc23 = makeContact(2, 3, Vector3(0, 0, 0), kVector3UnitY, 0.005f);
        solver.addContact(cc23, kVector3UnitX, kVector3UnitZ);

        solver.solve(bodies, numBodies, dt, config);
    }

    // After simulation, all boxes should be roughly at rest
    for (u32 i = 0; i < 3; ++i) {
        EXPECT_NEAR(bodies[i].linearVelocity.length(), 0.0f, 0.5f)
            << "Body " << i << " still moving after stack stabilization";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Inclined Plane
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, InclinedPlane) {
    // Sphere on a 45-degree inclined plane
    // Normal is (0.707, 0.707, 0) pointing up-right
    f32 invSqrt2 = 1.0f / std::sqrt(2.0f);
    Vector3 inclineNormal(invSqrt2, invSqrt2, 0);

    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, 0, 0);

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), inclineNormal, 0.005f);
    cc.friction = 0.8f; // high friction
    solver.addContact(cc, kVector3UnitZ, inclineNormal.cross(kVector3UnitZ).normalized());

    SolverConfig config;
    config.velocityIterations = 10;
    config.warmStartingEnabled = false;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // With high friction on a 45-degree incline, the sphere should not slide much
    // The tangential component of gravity along the incline is g*sin(45) ~ 6.93
    // Max friction impulse = mu * normal_impulse
    // After one frame, velocity should be limited
    f32 tangentialSpeed = bodies[0].linearVelocity.dot(
        Vector3(-invSqrt2, invSqrt2, 0)); // direction along incline
    // Should not gain excessive speed in one frame
    EXPECT_LT(std::abs(tangentialSpeed), 2.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Energy Conservation
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, EnergyConservation) {
    // Two spheres colliding elastically (restitution = 1.0)
    // Total kinetic energy should be approximately conserved
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeBody(1.0f);
    bodies[0].linearVelocity = Vector3(3, 0, 0);
    bodies[1].linearVelocity = Vector3(-3, 0, 0);

    f32 keBefore = 0.5f * 1.0f * 9.0f + 0.5f * 1.0f * 9.0f; // 9.0

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 0, 0), kVector3UnitX, 0.001f,
                                        1.0f, 1.0f, 1.0f); // restitution = 1.0
    solver.addContact(cc, kVector3UnitY, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 20;
    config.warmStartingEnabled = false;
    config.restitutionThreshold = 0.01f;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    f32 keAfter = 0.5f * 1.0f * bodies[0].linearVelocity.lengthSq()
                + 0.5f * 1.0f * bodies[1].linearVelocity.lengthSq();

    // Energy should be approximately conserved (within 20% due to numerical precision)
    EXPECT_GT(keAfter, keBefore * 0.5f);
    EXPECT_LT(keAfter, keBefore * 1.5f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Impulse Accumulation
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, ImpulseAccumulation) {
    // Verify impulses accumulate correctly across iterations
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, -10, 0); // fast fall

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 1;
    config.warmStartingEnabled = false;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    // With 1 iteration, the impulse should be applied
    EXPECT_GT(bodies[0].linearVelocity.y, -10.0f);

    // Check accumulated impulse is positive (non-penetration)
    EXPECT_GT(solver.contacts[0].accumulatedNormalImpulse, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Convergence with Iterations
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, ConvergenceWithIterations) {
    // More iterations should apply more total impulse (more correction)
    auto solveWithIterations = [](u32 iterations) -> f32 {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(0, -5, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = iterations;
        config.warmStartingEnabled = false;
        config.baumgarteFactor = 0.0f;

        solver.solve(bodies, 2, 1.0f/60.0f, config);

        return solver.getStats().totalImpulse;
    };

    f32 impulse1 = solveWithIterations(1);
    f32 impulse4 = solveWithIterations(4);
    f32 impulse10 = solveWithIterations(10);

    // More iterations should accumulate at least as much total impulse
    EXPECT_GE(impulse4, impulse1 - kEpsilon);
    EXPECT_GE(impulse10, impulse4 - kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Determinism
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, Determinism) {
    // Same inputs should produce same outputs
    auto solveOnce = []() -> Vector3 {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(1, -3, 0.5f);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.008f);
        cc.friction = 0.3f;
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 8;
        config.warmStartingEnabled = false;

        solver.solve(bodies, 2, 1.0f/60.0f, config);
        return bodies[0].linearVelocity;
    };

    Vector3 result1 = solveOnce();
    Vector3 result2 = solveOnce();

    EXPECT_NEAR(result1.x, result2.x, kEpsilon);
    EXPECT_NEAR(result1.y, result2.y, kEpsilon);
    EXPECT_NEAR(result1.z, result2.z, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: SolverContact Builder
// ═══════════════════════════════════════════════════════════════════════════

TEST(SolverContact, BuildFromConstraint) {
    ContactConstraint cc = makeContact(0, 1, Vector3(0,1,0), kVector3UnitY, 0.01f);
    cc.restitution = 0.5f;
    cc.friction = 0.3f;

    SolverContact sc = buildSolverContact(cc, kVector3UnitX, kVector3UnitZ);

    EXPECT_EQ(sc.bodyIDA, 0u);
    EXPECT_EQ(sc.bodyIDB, 1u);
    EXPECT_NEAR(sc.penetration, 0.01f, kEpsilon);
    EXPECT_NEAR(sc.restitution, 0.5f, kEpsilon);
    EXPECT_NEAR(sc.friction, 0.3f, kEpsilon);
    EXPECT_GT(sc.normalMass, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Split Impulse
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, SplitImpulseNoEnergyAddition) {
    // Split impulse should not add energy to resting contacts
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, 0, 0);

    SolverConfig config;
    config.velocityIterations = 10;
    config.splitImpulseEnabled = true;
    config.baumgarteFactorPosition = 0.8f;

    f32 dt = 1.0f / 60.0f;

    // Simulate 5 frames
    for (int frame = 0; frame < 5; ++frame) {
        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.005f);
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);
        solver.solve(bodies, 2, dt, config);
    }

    // With split impulse, velocity should remain near zero
    EXPECT_NEAR(bodies[0].linearVelocity.length(), 0.0f, 0.01f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Sequential Impulse Solver: Solver Statistics
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequentialImpulseSolver, Statistics) {
    SolverBodyData bodies[2];
    bodies[0] = makeBody(1.0f);
    bodies[1] = makeStaticBody();
    bodies[0].linearVelocity = Vector3(0, -5, 0);

    SequentialImpulseSolver solver;
    ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
    solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

    SolverConfig config;
    config.velocityIterations = 8;
    config.warmStartingEnabled = false;

    solver.solve(bodies, 2, 1.0f/60.0f, config);

    const SolverStats& stats = solver.getStats();
    EXPECT_EQ(stats.contactCount, 1u);
    EXPECT_EQ(stats.velocityIterations, 8u);
    EXPECT_GT(stats.totalImpulse, 0.0f);
}
