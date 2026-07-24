#include <gtest/gtest.h>

#include "foundation/core/types.hpp"
#include "foundation/time/clock.hpp"

#include <thread>

using namespace foundation;

namespace {

TEST(ClockTest, NowReturnsNonZero) {
    auto tp = now();
    EXPECT_GT(tp.time_since_epoch().count(), 0);
}

TEST(ClockTest, ElapsedSeconds) {
    auto start = now();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    f64 elapsed = elapsed_seconds(start);
    EXPECT_GE(elapsed, 0.04);
    EXPECT_LT(elapsed, 0.2);
}

TEST(ClockTest, ElapsedMilliseconds) {
    auto start = now();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    f64 elapsed = elapsed_milliseconds(start);
    EXPECT_GE(elapsed, 40.0);
    EXPECT_LT(elapsed, 200.0);
}

TEST(StopwatchTest, StartStop) {
    Stopwatch sw;
    EXPECT_TRUE(sw.is_running());

    sw.stop();
    EXPECT_FALSE(sw.is_running());

    sw.start();
    EXPECT_TRUE(sw.is_running());
}

TEST(StopwatchTest, Reset) {
    Stopwatch sw;
    sw.stop();
    sw.reset();
    EXPECT_FALSE(sw.is_running());
    EXPECT_GE(sw.elapsed_seconds(), 0.0);
}

TEST(StopwatchTest, ElapsedTime) {
    Stopwatch sw;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    f64 elapsed = sw.elapsed_seconds();
    EXPECT_GE(elapsed, 0.04);
    EXPECT_LT(elapsed, 0.5);
    sw.stop();
}

TEST(DeltaTimerTest, TickReturnsDelta) {
    DeltaTimer timer;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    f64 delta = timer.tick();
    EXPECT_GE(delta, 0.0);
    EXPECT_LT(delta, 0.5);
}

TEST(DeltaTimerTest, Reset) {
    DeltaTimer timer;
    (void)timer.tick();
    timer.reset();
    f64 delta = timer.last_delta();
    EXPECT_GE(delta, 0.0);
}

} // namespace
