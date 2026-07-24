#include <gtest/gtest.h>

#include "foundation/core/types.hpp"
#include "foundation/threading/thread.hpp"
#include "foundation/time/clock.hpp"

#include <atomic>
#include <chrono>

using namespace foundation;

namespace {

TEST(ThreadTest, Construction) {
    std::atomic<bool> executed{false};
    Thread thread([&executed]() {
        executed.store(true);
    }, "test_thread");

    thread.join();
    EXPECT_TRUE(executed.load());
}

TEST(ThreadTest, Joinable) {
    Thread thread([]() {}, "test");
    EXPECT_TRUE(thread.joinable());
    thread.join();
    EXPECT_FALSE(thread.joinable());
}

TEST(ThreadTest, IsRunning) {
    std::atomic<bool> keep_running{true};
    Thread thread([&keep_running]() {
        while (keep_running.load()) {
            yield();
        }
    }, "running_test");

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(thread.is_running());

    keep_running.store(false);
    thread.join();
    EXPECT_FALSE(thread.is_running());
}

TEST(ThreadTest, Name) {
    Thread thread([]() {}, "my_thread");
    EXPECT_EQ(thread.name(), "my_thread");
    thread.join();
}

TEST(ThreadTest, MoveConstruction) {
    Thread t1([]() {}, "movable");
    Thread t2(std::move(t1));
    EXPECT_EQ(t2.name(), "movable");
    t2.join();
}

TEST(ThreadTest, HardwareConcurrency) {
    u32 cores = Thread::hardware_concurrency();
    EXPECT_GT(cores, 0u);
}

TEST(ThreadTest, SleepSeconds) {
    auto start = now();
    sleep_seconds(0.01);
    f64 elapsed = elapsed_seconds(start);
    EXPECT_GE(elapsed, 0.005);
}

TEST(ThreadTest, Yield) {
    yield();
}

} // namespace
