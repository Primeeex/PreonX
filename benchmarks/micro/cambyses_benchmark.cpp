#include <benchmark/benchmark.h>

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

static void BM_EntityCreation(benchmark::State& state) {
    for (auto _ : state) {
        World world;
        for (int i = 0; i < state.range(0); ++i) {
            benchmark::DoNotOptimize(world.create());
        }
    }
    state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_EntityCreation)->Range(8, 1 << 16)->Complexity();

static void BM_EntityDestruction(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        World world;
        foundation::DynamicArray<Entity> entities;
        for (int i = 0; i < state.range(0); ++i) {
            entities.push_back(world.create());
        }
        state.ResumeTiming();

        for (auto& e : entities) {
            world.destroy(e);
        }
    }
    state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_EntityDestruction)->Range(8, 1 << 16)->Complexity();

static void BM_ComponentInsertion(benchmark::State& state) {
    for (auto _ : state) {
        World world;
        for (int i = 0; i < state.range(0); ++i) {
            Entity e = world.create();
            benchmark::DoNotOptimize(e);
            world.add_component(e, Position{1.0f, 2.0f});
            world.add_component(e, Velocity{0.1f, 0.2f});
        }
    }
    state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_ComponentInsertion)->Range(8, 1 << 16)->Complexity();

static void BM_ComponentRemoval(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        World world;
        foundation::DynamicArray<Entity> entities;
        for (int i = 0; i < state.range(0); ++i) {
            Entity e = world.create();
            world.add_component(e, Position{1.0f, 2.0f});
            world.add_component(e, Velocity{0.1f, 0.2f});
            entities.push_back(e);
        }
        state.ResumeTiming();

        for (auto& e : entities) {
            world.remove_component<Velocity>(e);
        }
    }
    state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_ComponentRemoval)->Range(8, 1 << 16)->Complexity();

static void BM_QueryIteration(benchmark::State& state) {
    World world;
    const auto N = state.range(0);
    for (int i = 0; i < N; ++i) {
        Entity e = world.create();
        world.add_component(e, Position{static_cast<float>(i), 0.0f});
        if (i % 2 == 0) {
            world.add_component(e, Velocity{1.0f, 0.0f});
        }
    }

    for (auto _ : state) {
        world.query<Position, Velocity>().each([](Entity, Position& pos, const Velocity& vel) {
            pos.x += vel.dx;
            benchmark::DoNotOptimize(pos.x);
        });
    }
    state.SetComplexityN(N);
}
BENCHMARK(BM_QueryIteration)->Range(8, 1 << 16)->Complexity();

} // namespace
