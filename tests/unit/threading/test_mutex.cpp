#include <gtest/gtest.h>

#include "foundation/threading/mutex.hpp"

#include <thread>
#include <vector>

namespace {

TEST(MutexTest, LockUnlock) {
    foundation::Mutex mutex;
    mutex.lock();
    mutex.unlock();
}

TEST(MutexTest, TryLock) {
    foundation::Mutex mutex;
    EXPECT_TRUE(mutex.try_lock());
    mutex.unlock();
}

TEST(MutexTest, ScopedLock) {
    foundation::Mutex mutex;
    {
        foundation::LockGuard lock(mutex);
        // Lock is held here
    }
    EXPECT_TRUE(mutex.try_lock());
    mutex.unlock();
}

TEST(MutexTest, ConcurrentIncrement) {
    foundation::Mutex mutex;
    int counter = 0;
    constexpr int kIterations = 10000;

    auto worker = [&]() {
        for (int i = 0; i < kIterations; ++i) {
            foundation::LockGuard lock(mutex);
            ++counter;
        }
    };

    std::thread t1(worker);
    std::thread t2(worker);
    t1.join();
    t2.join();

    EXPECT_EQ(counter, 2 * kIterations);
}

TEST(RecursiveMutexTest, RecursiveLock) {
    foundation::RecursiveMutex mutex;
    mutex.lock();
    mutex.lock(); // Should not deadlock
    mutex.unlock();
    mutex.unlock();
}

TEST(SharedMutexTest, SharedLock) {
    foundation::SharedMutex mutex;
    {
        foundation::SharedLockGuard lock(mutex);
        // Shared lock held
    }
    EXPECT_TRUE(mutex.try_lock());
    mutex.unlock();
}

} // namespace
