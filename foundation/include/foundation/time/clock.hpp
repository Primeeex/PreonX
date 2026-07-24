#pragma once

#include "foundation/core/types.hpp"

#include <chrono>

namespace foundation {

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
using Duration = std::chrono::duration<f64>;

[[nodiscard]] inline TimePoint now() noexcept {
    return SteadyClock::now();
}

[[nodiscard]] inline f64 elapsed_seconds(TimePoint start, TimePoint end = now()) noexcept {
    return std::chrono::duration<f64>(end - start).count();
}

[[nodiscard]] inline f64 elapsed_milliseconds(TimePoint start, TimePoint end = now()) noexcept {
    return std::chrono::duration<f64, std::milli>(end - start).count();
}

[[nodiscard]] inline f64 elapsed_microseconds(TimePoint start, TimePoint end = now()) noexcept {
    return std::chrono::duration<f64, std::micro>(end - start).count();
}

class Stopwatch {
public:
    Stopwatch() : start_(now()), running_(true) {}

    void start() {
        start_ = now();
        running_ = true;
    }

    void stop() {
        if (running_) {
            accumulated_ += Duration(now() - start_);
            running_ = false;
        }
    }

    void reset() {
        accumulated_ = Duration::zero();
        running_ = false;
    }

    [[nodiscard]] f64 elapsed_seconds() const {
        if (running_) {
            return accumulated_.count() + foundation::elapsed_seconds(start_);
        }
        return accumulated_.count();
    }

    [[nodiscard]] f64 elapsed_milliseconds() const {
        return elapsed_seconds() * 1000.0;
    }

    [[nodiscard]] bool is_running() const noexcept {
        return running_;
    }

private:
    TimePoint start_;
    Duration accumulated_ = Duration::zero();
    bool running_ = false;
};

class DeltaTimer {
public:
    DeltaTimer() : last_(now()) {}

    [[nodiscard]] f64 tick() {
        TimePoint current = now();
        f64 delta = foundation::elapsed_seconds(last_, current);
        last_ = current;
        return delta;
    }

    [[nodiscard]] f64 last_delta() const noexcept {
        return last_delta_;
    }

    void reset() {
        last_ = now();
        last_delta_ = 0.0;
    }

private:
    TimePoint last_;
    f64 last_delta_ = 0.0;
};

} // namespace foundation
