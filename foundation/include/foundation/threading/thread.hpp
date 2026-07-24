#pragma once

#include "foundation/core/types.hpp"
#include "foundation/threading/mutex.hpp"

#include <functional>
#include <string>
#include <thread>

namespace foundation {

class Thread {
public:
    Thread() = default;

    explicit Thread(std::function<void()> func, std::string name = {});

    ~Thread();

    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    Thread(Thread&& other) noexcept;
    Thread& operator=(Thread&& other) noexcept;

    void join();
    [[nodiscard]] bool joinable() const noexcept;
    [[nodiscard]] bool is_running() const noexcept;
    [[nodiscard]] const std::string& name() const noexcept;

    [[nodiscard]] static u32 hardware_concurrency() noexcept;
    [[nodiscard]] static u32 current_thread_id() noexcept;

private:
    std::thread thread_;
    std::string name_;
    bool running_ = false;
    Mutex mutex_;
};

/// Sleep the current thread for the given number of seconds.
void sleep_seconds(f64 seconds);

/// Yield the current thread's time slice.
void yield() noexcept;

} // namespace foundation
