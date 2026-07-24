#pragma once

#include "foundation/core/types.hpp"

#include <mutex>
#include <shared_mutex>

namespace foundation {

class Mutex {
public:
    Mutex() = default;
    ~Mutex() = default;

    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;
    Mutex(Mutex&&) = delete;
    Mutex& operator=(Mutex&&) = delete;

    void lock() {
        mutex_.lock();
    }

    bool try_lock() {
        return mutex_.try_lock();
    }

    void unlock() {
        mutex_.unlock();
    }

    [[nodiscard]] std::mutex& native_handle() noexcept {
        return mutex_;
    }

    [[nodiscard]] const std::mutex& native_handle() const noexcept {
        return mutex_;
    }

private:
    std::mutex mutex_;
};

class RecursiveMutex {
public:
    RecursiveMutex() = default;
    ~RecursiveMutex() = default;

    RecursiveMutex(const RecursiveMutex&) = delete;
    RecursiveMutex& operator=(const RecursiveMutex&) = delete;
    RecursiveMutex(RecursiveMutex&&) = delete;
    RecursiveMutex& operator=(RecursiveMutex&&) = delete;

    void lock() {
        mutex_.lock();
    }

    bool try_lock() {
        return mutex_.try_lock();
    }

    void unlock() {
        mutex_.unlock();
    }

    [[nodiscard]] std::recursive_mutex& native_handle() noexcept {
        return mutex_;
    }

private:
    std::recursive_mutex mutex_;
};

class SharedMutex {
public:
    SharedMutex() = default;
    ~SharedMutex() = default;

    SharedMutex(const SharedMutex&) = delete;
    SharedMutex& operator=(const SharedMutex&) = delete;
    SharedMutex(SharedMutex&&) = delete;
    SharedMutex& operator=(SharedMutex&&) = delete;

    void lock() {
        mutex_.lock();
    }

    bool try_lock() {
        return mutex_.try_lock();
    }

    void unlock() {
        mutex_.unlock();
    }

    void lock_shared() {
        mutex_.lock_shared();
    }

    bool try_lock_shared() {
        return mutex_.try_lock_shared();
    }

    void unlock_shared() {
        mutex_.unlock_shared();
    }

    [[nodiscard]] std::shared_mutex& native_handle() noexcept {
        return mutex_;
    }

private:
    std::shared_mutex mutex_;
};

template <typename MutexType>
class ScopedLock {
public:
    explicit ScopedLock(MutexType& mutex) : mutex_(mutex) {
        mutex_.lock();
    }

    ~ScopedLock() {
        mutex_.unlock();
    }

    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
    ScopedLock(ScopedLock&&) = delete;
    ScopedLock& operator=(ScopedLock&&) = delete;

private:
    MutexType& mutex_;
};

template <typename MutexType>
class ScopedSharedLock {
public:
    explicit ScopedSharedLock(MutexType& mutex) : mutex_(mutex) {
        mutex_.lock_shared();
    }

    ~ScopedSharedLock() {
        mutex_.unlock_shared();
    }

    ScopedSharedLock(const ScopedSharedLock&) = delete;
    ScopedSharedLock& operator=(const ScopedSharedLock&) = delete;
    ScopedSharedLock(ScopedSharedLock&&) = delete;
    ScopedSharedLock& operator=(ScopedSharedLock&&) = delete;

private:
    MutexType& mutex_;
};

using LockGuard = ScopedLock<Mutex>;
using RecursiveLockGuard = ScopedLock<RecursiveMutex>;
using SharedLockGuard = ScopedSharedLock<SharedMutex>;

} // namespace foundation
