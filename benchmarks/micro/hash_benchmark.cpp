#include <benchmark/benchmark.h>

#include "foundation/hashing/hash.hpp"

#include <string>
#include <cstring>

static void BM_HashString_Small(benchmark::State& state) {
    for (auto _ : state) {
        foundation::HashValue h = foundation::hash::hash_string("hello world");
        benchmark::DoNotOptimize(h);
    }
}
BENCHMARK(BM_HashString_Small);

static void BM_HashString_Medium(benchmark::State& state) {
    std::string data(256, 'a');
    for (auto _ : state) {
        foundation::HashValue h = foundation::hash::hash_string(data);
        benchmark::DoNotOptimize(h);
    }
}
BENCHMARK(BM_HashString_Medium);

static void BM_HashString_Large(benchmark::State& state) {
    std::string data(8192, 'x');
    for (auto _ : state) {
        foundation::HashValue h = foundation::hash::hash_string(data);
        benchmark::DoNotOptimize(h);
    }
}
BENCHMARK(BM_HashString_Large);

static void BM_HashBytes_64(benchmark::State& state) {
    alignas(64) foundation::u8 data[64];
    std::memset(data, 0xAB, 64);
    for (auto _ : state) {
        foundation::HashValue h = foundation::hash::hash_bytes(data, 64);
        benchmark::DoNotOptimize(h);
    }
}
BENCHMARK(BM_HashBytes_64);

static void BM_HashBytes_1024(benchmark::State& state) {
    std::vector<foundation::u8> data(1024, 0xCD);
    for (auto _ : state) {
        foundation::HashValue h = foundation::hash::hash_bytes(data.data(), data.size());
        benchmark::DoNotOptimize(h);
    }
}
BENCHMARK(BM_HashBytes_1024);

static void BM_HashCombine(benchmark::State& state) {
    for (auto _ : state) {
        foundation::HashValue h = foundation::hash::combine({100, 200, 300, 400, 500});
        benchmark::DoNotOptimize(h);
    }
}
BENCHMARK(BM_HashCombine);
