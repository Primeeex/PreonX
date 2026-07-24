#include <benchmark/benchmark.h>
#include <primeon/solver/solver_config.hpp>
#include <primeon/solver/sequential_impulse/sequential_impulse_solver.hpp>
#include <primeon/solver/friction/friction_solver.hpp>
#include <primeon/solver/restitution/restitution_solver.hpp>
#include <primeon/solver/stabilization/stabilization.hpp>
#include <primeon/constraints/contact_constraint.hpp>

using namespace primeon::math;

// ── Helpers ─────────────────────────────────────────────────────────────────

static SolverBodyData makeBody(f32 invMass) {
    SolverBodyData b;
    b.massData.inverseMass = invMass;
    b.massData.inverseInertiaDiag = Vector3(1, 1, 1);
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
    cc.massA.inverseInertiaDiag = Vector3(1, 1, 1);
    cc.massB.inverseMass = invMassB;
    cc.massB.inverseInertiaDiag = Vector3(1, 1, 1);
    cc.pointA = point;
    cc.pointB = point;
    cc.normal = normal;
    cc.penetration = penetration;
    cc.restitution = restitution;
    cc.friction = friction;
    return cc;
}

// ── Single Contact Benchmarks ───────────────────────────────────────────────

static void BM_Solver_SingleContact_1Iter(benchmark::State& state) {
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(0, -5, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 1;
        config.warmStartingEnabled = false;
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_SingleContact_1Iter);

static void BM_Solver_SingleContact_8Iter(benchmark::State& state) {
    for (auto _ : state) {
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
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_SingleContact_8Iter);

static void BM_Solver_SingleContact_20Iter(benchmark::State& state) {
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(0, -5, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 20;
        config.warmStartingEnabled = false;
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_SingleContact_20Iter);

// ── Friction Benchmarks ─────────────────────────────────────────────────────

static void BM_Solver_SingleContactWithFriction(benchmark::State& state) {
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(5, -2, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.05f);
        cc.friction = 0.5f;
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 8;
        config.warmStartingEnabled = false;
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_SingleContactWithFriction);

static void BM_Solver_SingleContactNoFriction(benchmark::State& state) {
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(5, -2, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.05f);
        cc.friction = 0.0f;
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 8;
        config.warmStartingEnabled = false;
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_SingleContactNoFriction);

// ── Restitution Benchmark ───────────────────────────────────────────────────

static void BM_Solver_RestitutionBounce(benchmark::State& state) {
    for (auto _ : state) {
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
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_RestitutionBounce);

// ── Multi-Contact (Box Stack) Benchmarks ────────────────────────────────────

static void BM_Solver_BoxStack_3Bodies(benchmark::State& state) {
    for (auto _ : state) {
        const u32 numBodies = 4;
        SolverBodyData bodies[numBodies];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeBody(1.0f);
        bodies[2] = makeBody(1.0f);
        bodies[3] = makeStaticBody();

        for (u32 i = 0; i < 3; ++i) {
            bodies[i].linearVelocity = Vector3(0, -1, 0);
        }

        SequentialImpulseSolver solver;
        solver.addContact(makeContact(0, 1, Vector3(0, 2, 0), kVector3UnitY, 0.005f),
                          kVector3UnitX, kVector3UnitZ);
        solver.addContact(makeContact(1, 2, Vector3(0, 1, 0), kVector3UnitY, 0.005f),
                          kVector3UnitX, kVector3UnitZ);
        solver.addContact(makeContact(2, 3, Vector3(0, 0, 0), kVector3UnitY, 0.005f),
                          kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 8;
        solver.solve(bodies, numBodies, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_BoxStack_3Bodies);

static void BM_Solver_BoxStack_8Bodies(benchmark::State& state) {
    for (auto _ : state) {
        const u32 numBodies = 9;
        SolverBodyData bodies[numBodies];
        for (u32 i = 0; i < 8; ++i) {
            bodies[i] = makeBody(1.0f);
            bodies[i].linearVelocity = Vector3(0, -1, 0);
        }
        bodies[8] = makeStaticBody();

        SequentialImpulseSolver solver;
        for (u32 i = 0; i < 8; ++i) {
            solver.addContact(
                makeContact(i, i + 1, Vector3(0, static_cast<f32>(i), 0),
                            kVector3UnitY, 0.005f),
                kVector3UnitX, kVector3UnitZ);
        }

        SolverConfig config;
        config.velocityIterations = 10;
        solver.solve(bodies, numBodies, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_BoxStack_8Bodies);

// ── Large Contact Set Benchmark ─────────────────────────────────────────────

static void BM_Solver_LargeContactSet(benchmark::State& state) {
    const u32 numContacts = 32;
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(0, -3, 0);

        SequentialImpulseSolver solver;
        for (u32 i = 0; i < numContacts; ++i) {
            f32 y = static_cast<f32>(i) * 0.1f;
            ContactConstraint cc = makeContact(0, 1, Vector3(0, y, 0), kVector3UnitY, 0.005f);
            cc.friction = 0.3f;
            solver.addContact(cc, kVector3UnitX, kVector3UnitZ);
        }

        SolverConfig config;
        config.velocityIterations = 8;
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_LargeContactSet);

// ── Warm Starting Effectiveness ─────────────────────────────────────────────

static void BM_Solver_WarmStart(benchmark::State& state) {
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(0, -5, 0);

        SequentialImpulseSolver solver;
        ContactConstraint cc = makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f);
        cc.accumulatedNormalImpulse = 4.0f; // previous frame's impulse
        solver.addContact(cc, kVector3UnitX, kVector3UnitZ);

        SolverConfig config;
        config.velocityIterations = 8;
        config.warmStartingEnabled = true;
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_WarmStart);

static void BM_Solver_NoWarmStart(benchmark::State& state) {
    for (auto _ : state) {
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
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
    }
}
BENCHMARK(BM_Solver_NoWarmStart);

// ── Split Impulse Benchmark ─────────────────────────────────────────────────

static void BM_Solver_SplitImpulse(benchmark::State& state) {
    for (auto _ : state) {
        SolverBodyData bodies[2];
        bodies[0] = makeBody(1.0f);
        bodies[1] = makeStaticBody();
        bodies[0].linearVelocity = Vector3(0, -3, 0);

        SequentialImpulseSolver solver;
        solver.addContact(
            makeContact(0, 1, Vector3(0, 1, 0), kVector3UnitY, 0.01f),
            kVector3UnitX, kVector3UnitZ);

        SolverConfig config = SolverConfig::simulation();
        solver.solve(bodies, 2, 1.0f / 60.0f, config);
        benchmark::DoNotOptimize(bodies);
        benchmark::DoNotOptimize(bodies[0].linearVelocity);
    }
}
BENCHMARK(BM_Solver_SplitImpulse);
