#include <benchmark/benchmark.h>
#include <primeon/world/physics_world.hpp>

using namespace primeon::math;

// ── Body Creation ───────────────────────────────────────────────────────────

static void BM_World_CreateDynamicBody(benchmark::State& state) {
    const u32 maxBodies = static_cast<u32>(state.range(0));
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = maxBodies;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        for (u32 i = 0; i < maxBodies; ++i) {
            BodyID id = world.createBody(BodyDescriptor::dynamicBody(
                Vector3(static_cast<f32>(i) * 2.0f, 0, 0), 1.0f));
            benchmark::DoNotOptimize(id);
        }
    }
}
BENCHMARK(BM_World_CreateDynamicBody)->Arg(16)->Arg(64)->Arg(256)->Arg(512);

static void BM_World_CreateStaticBody(benchmark::State& state) {
    const u32 maxBodies = static_cast<u32>(state.range(0));
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = maxBodies;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        for (u32 i = 0; i < maxBodies; ++i) {
            BodyID id = world.createBody(BodyDescriptor::staticBody(
                Vector3(static_cast<f32>(i) * 2.0f, 0, 0)));
            benchmark::DoNotOptimize(id);
        }
    }
}
BENCHMARK(BM_World_CreateStaticBody)->Arg(16)->Arg(64)->Arg(256)->Arg(512);

// ── Full World Step (widely spaced — no broadphase pairs) ──────────────────

static void BM_World_StepEmpty(benchmark::State& state) {
    const u32 maxBodies = static_cast<u32>(state.range(0));
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = maxBodies;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);
        benchmark::DoNotOptimize(world);
        world.step(1.0f / 60.0f);
    }
}
BENCHMARK(BM_World_StepEmpty)->Arg(16)->Arg(64)->Arg(256)->Arg(512);

static void BM_World_StepNoContacts(benchmark::State& state) {
    const u32 numBodies = static_cast<u32>(state.range(0));
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = numBodies;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        for (u32 i = 0; i < numBodies; ++i) {
            BodyID id = world.createBody(BodyDescriptor::dynamicBody(
                Vector3(static_cast<f32>(i) * 100.0f, 0, 0), 1.0f));
            benchmark::DoNotOptimize(id);
        }

        world.step(1.0f / 60.0f);
    }
}
BENCHMARK(BM_World_StepNoContacts)->Arg(16)->Arg(64)->Arg(256)->Arg(512);

// ── Sleeping ────────────────────────────────────────────────────────────────

static void BM_World_SleepTransition(benchmark::State& state) {
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = 16;
        cfg.sleepingEnabled = true;
        cfg.gravity = Vector3(0, 0, 0);
        cfg.sleep.linearSleepThreshold = 0.01f;
        cfg.sleep.sleepTimeThreshold = 30;
        PhysicsWorld world(cfg);

        BodyID body = world.createBody(
            BodyDescriptor::dynamicBody(Vector3(0, 0, 0), 1.0f));
        world.applyImpulse(body, Vector3(0, 1, 0));

        for (u32 i = 0; i < 120; ++i) {
            world.step(1.0f / 60.0f);
        }
    }
}
BENCHMARK(BM_World_SleepTransition);

static void BM_World_StepSleepingWorld(benchmark::State& state) {
    WorldConfig cfg;
    cfg.maxBodies = 16;
    cfg.sleepingEnabled = true;
    cfg.gravity = Vector3(0, 0, 0);
    PhysicsWorld world(cfg);

    BodyID body = world.createBody(
        BodyDescriptor::dynamicBody(Vector3(0, 0, 0), 1.0f));
    world.applyImpulse(body, Vector3(0, 1, 0));

    for (u32 i = 0; i < 120; ++i) {
        world.step(1.0f / 60.0f);
    }

    for (auto _ : state) {
        world.step(1.0f / 60.0f);
    }
}
BENCHMARK(BM_World_StepSleepingWorld);

// ── Fixed Timestep ──────────────────────────────────────────────────────────

static void BM_World_FixedTimestepSubsteps(benchmark::State& state) {
    const f32 frameTime = static_cast<f32>(state.range(0)) / 100.0f;
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = 16;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        BodyID id = world.createBody(BodyDescriptor::dynamicBody(Vector3(0, 10, 0), 1.0f));
        benchmark::DoNotOptimize(id);
        world.step(frameTime);
    }
}
BENCHMARK(BM_World_FixedTimestepSubsteps)
    ->Arg(5)->Arg(16)->Arg(33)->Arg(100);

// ── Island Building (widely spaced) ────────────────────────────────────────

static void BM_World_IslandBuilding_NoContacts(benchmark::State& state) {
    const u32 numBodies = static_cast<u32>(state.range(0));
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = numBodies;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        for (u32 i = 0; i < numBodies; ++i) {
            BodyID id = world.createBody(BodyDescriptor::dynamicBody(
                Vector3(static_cast<f32>(i) * 100.0f, 0, 0), 1.0f));
            benchmark::DoNotOptimize(id);
        }

        world.step(1.0f / 60.0f);
    }
}
BENCHMARK(BM_World_IslandBuilding_NoContacts)->Arg(16)->Arg(64)->Arg(256)->Arg(512);

// ── Single Body Physics ─────────────────────────────────────────────────────

static void BM_World_SingleBodyGravity(benchmark::State& state) {
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = 16;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        BodyID id = world.createBody(
            BodyDescriptor::dynamicBody(Vector3(0, 10, 0), 1.0f));
        benchmark::DoNotOptimize(id);

        world.step(1.0f / 60.0f);
    }
}
BENCHMARK(BM_World_SingleBodyGravity);

// ── Multiple Steps ──────────────────────────────────────────────────────────

static void BM_World_10Steps_SingleBody(benchmark::State& state) {
    for (auto _ : state) {
        WorldConfig cfg;
        cfg.maxBodies = 16;
        cfg.sleepingEnabled = false;
        PhysicsWorld world(cfg);

        BodyID id = world.createBody(
            BodyDescriptor::dynamicBody(Vector3(0, 100, 0), 1.0f));
        benchmark::DoNotOptimize(id);

        for (u32 i = 0; i < 10; ++i) {
            world.step(1.0f / 60.0f);
        }
    }
}
BENCHMARK(BM_World_10Steps_SingleBody);

// ── Statistics Query ────────────────────────────────────────────────────────

static void BM_World_StatsQuery(benchmark::State& state) {
    WorldConfig cfg;
    cfg.maxBodies = 256;
    cfg.sleepingEnabled = false;
    PhysicsWorld world(cfg);

    for (u32 i = 0; i < 256; ++i) {
        BodyID id = world.createBody(BodyDescriptor::dynamicBody(
            Vector3(static_cast<f32>(i) * 2.0f, 0, 0), 1.0f));
        benchmark::DoNotOptimize(id);
    }

    world.step(1.0f / 60.0f);

    for (auto _ : state) {
        const auto& stats = world.getStats();
        benchmark::DoNotOptimize(&stats);
        u32 count = world.getBodyCount();
        benchmark::DoNotOptimize(count);
    }
}
BENCHMARK(BM_World_StatsQuery);
