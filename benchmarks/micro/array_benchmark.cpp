#include <benchmark/benchmark.h>

#include "foundation/containers/dynamic_array.hpp"

static void BM_ArrayPushBack(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        foundation::DynamicArray<int> arr;
        state.ResumeTiming();

        for (int i = 0; i < state.range(0); ++i) {
            arr.push_back(i);
        }
        benchmark::DoNotOptimize(arr.data());
    }
}
BENCHMARK(BM_ArrayPushBack)->Range(8, 1 << 16);

static void BM_ArrayAccess(benchmark::State& state) {
    foundation::DynamicArray<int> arr;
    for (int i = 0; i < state.range(0); ++i) {
        arr.push_back(i);
    }

    for (auto _ : state) {
        for (size_t i = 0; i < arr.size(); ++i) {
            benchmark::DoNotOptimize(arr[i]);
        }
    }
}
BENCHMARK(BM_ArrayAccess)->Range(8, 1 << 16);

static void BM_ArrayIteration(benchmark::State& state) {
    foundation::DynamicArray<int> arr;
    for (int i = 0; i < state.range(0); ++i) {
        arr.push_back(i);
    }

    for (auto _ : state) {
        int sum = 0;
        for (int val : arr) {
            sum += val;
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(BM_ArrayIteration)->Range(8, 1 << 16);

static void BM_ArrayCopy(benchmark::State& state) {
    foundation::DynamicArray<int> original;
    for (int i = 0; i < state.range(0); ++i) {
        original.push_back(i);
    }

    for (auto _ : state) {
        foundation::DynamicArray<int> copy = original;
        benchmark::DoNotOptimize(copy.data());
    }
}
BENCHMARK(BM_ArrayCopy)->Range(8, 1 << 16);

static void BM_ArrayInsertMiddle(benchmark::State& state) {
    for (auto _ : state) {
        state.PauseTiming();
        foundation::DynamicArray<int> arr;
        for (int i = 0; i < state.range(0); ++i) {
            arr.push_back(i);
        }
        state.ResumeTiming();

        arr.insert(arr.begin() + arr.size() / 2, 999);
    }
}
BENCHMARK(BM_ArrayInsertMiddle)->Range(8, 1 << 14);
