#include <benchmark/benchmark.h>

#include "foundation/memory/allocator.hpp"

#include <cstdlib>
#include <cstring>

static void BM_AllocateDeallocate(benchmark::State& state) {
    for (auto _ : state) {
        void* ptr = foundation::memory_allocate(128);
        benchmark::DoNotOptimize(ptr);
        foundation::memory_deallocate(ptr, 128);
    }
}
BENCHMARK(BM_AllocateDeallocate)->Iterations(100000);

static void BM_AllocateSmall(benchmark::State& state) {
    for (auto _ : state) {
        void* ptr = foundation::memory_allocate(32);
        benchmark::DoNotOptimize(ptr);
        foundation::memory_deallocate(ptr, 32);
    }
}
BENCHMARK(BM_AllocateSmall)->Iterations(100000);

static void BM_AllocateLarge(benchmark::State& state) {
    for (auto _ : state) {
        void* ptr = foundation::memory_allocate(1024 * 1024);
        benchmark::DoNotOptimize(ptr);
        foundation::memory_deallocate(ptr, 1024 * 1024);
    }
}
BENCHMARK(BM_AllocateLarge)->Iterations(10000);

static void BM_AllocateZero(benchmark::State& state) {
    for (auto _ : state) {
        void* ptr = foundation::memory_allocate_zero(256);
        benchmark::DoNotOptimize(ptr);
        foundation::memory_deallocate(ptr, 256);
    }
}
BENCHMARK(BM_AllocateZero)->Iterations(100000);

static void BM_AllocateAligned64(benchmark::State& state) {
    for (auto _ : state) {
        void* ptr = foundation::memory_allocate(128, 64);
        benchmark::DoNotOptimize(ptr);
        foundation::memory_deallocate(ptr, 128, 64);
    }
}
BENCHMARK(BM_AllocateAligned64)->Iterations(100000);
