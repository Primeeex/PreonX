#include "foundation/threading/thread.hpp"

namespace foundation {

Thread::Thread(std::function<void()> func, std::string name) : name_(std::move(name)), running_(true) {
    thread_ = std::thread([this, func = std::move(func)]() {
        func();
        LockGuard lock(mutex_);
        running_ = false;
    });
}

Thread::~Thread() {
    if (thread_.joinable()) {
        thread_.detach();
    }
}

Thread::Thread(Thread&& other) noexcept
    : thread_(std::move(other.thread_)), name_(std::move(other.name_)), running_(other.running_) {
    other.running_ = false;
}

Thread& Thread::operator=(Thread&& other) noexcept {
    if (this != &other) {
        if (thread_.joinable()) {
            thread_.detach();
        }
        thread_ = std::move(other.thread_);
        name_ = std::move(other.name_);
        running_ = other.running_;
        other.running_ = false;
    }
    return *this;
}

void Thread::join() {
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool Thread::joinable() const noexcept {
    return thread_.joinable();
}

bool Thread::is_running() const noexcept {
    LockGuard lock(const_cast<Mutex&>(mutex_));
    return running_;
}

const std::string& Thread::name() const noexcept {
    return name_;
}

u32 Thread::hardware_concurrency() noexcept {
    return std::thread::hardware_concurrency();
}

u32 Thread::current_thread_id() noexcept {
    static thread_local u32 id = [] {
        auto handle = std::this_thread::get_id();
        return static_cast<u32>(std::hash<std::thread::id>{}(handle));
    }();
    return id;
}

void sleep_seconds(f64 seconds) {
    std::this_thread::sleep_for(std::chrono::duration<f64>(seconds));
}

void yield() noexcept {
    std::this_thread::yield();
}

} // namespace foundation
