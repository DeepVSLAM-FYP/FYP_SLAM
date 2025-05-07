//utils/ThreadSafeQueue.h
#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <stdexcept>

template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(std::size_t max_size = 20)
        : buffer_(max_size),
          max_size_(max_size),
          head_(0), tail_(0), count_(0),
          shutdown_(false)
    {}


    // non-copyable
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    // --- Producer API ---
    void enqueue(const T& item)            { emplace(item); }
    void enqueue(T&& item)                 { emplace(std::move(item)); }

    template<typename Rep, typename Period>
    bool try_enqueue_for(const T& item,
                         const std::chrono::duration<Rep,Period>& dur)
    {
        return try_emplace_for(dur, item);
    }

    template<typename Rep, typename Period>
    bool try_enqueue_for(T&& item,
                         const std::chrono::duration<Rep,Period>& dur)
    {
        return try_emplace_for(dur, std::move(item));
    }

    // --- Consumer API ---
    bool dequeue(T& out) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_not_empty_.wait(lock, [this]() {
            return count_ > 0 || shutdown_;
        });
        // if shutdown & empty, signal "no more data"
        if (shutdown_ && count_ == 0)
            return false;

        out = std::move(buffer_[head_]);
        head_ = (head_ + 1) % max_size_;
        --count_;
        lock.unlock();
        cond_not_full_.notify_one();
        return true;
    }

    template<typename Rep, typename Period>
    bool try_dequeue_for(T& out,
                         const std::chrono::duration<Rep,Period>& dur)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_not_empty_.wait_for(lock, dur, [this]() {
                return count_ > 0 || shutdown_;
            }))
            return false;

        if (shutdown_ && count_ == 0)
            return false;

        out = std::move(buffer_[head_]);
        head_ = (head_ + 1) % max_size_;
        --count_;
        lock.unlock();
        cond_not_full_.notify_one();
        return true;
    }

    // --- Shutdown & Introspection ---
    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        cond_not_empty_.notify_all();
        cond_not_full_.notify_all();
    }

    bool is_shutdown() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return shutdown_;
    }

    int size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(count_);
    }

private:
    // exactly one mutex + two condvars
    mutable std::mutex              mutex_;
    std::condition_variable         cond_not_empty_;
    std::condition_variable         cond_not_full_;

    // ring buffer state
    std::vector<T>                  buffer_;
    const std::size_t               max_size_;
    std::size_t                     head_, tail_, count_;

    bool                            shutdown_;

    // helper to emplace a new item (blocking)
    template<typename U>
    void emplace(U&& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_not_full_.wait(lock, [this]() {
            return count_ < max_size_ || shutdown_;
        });
        if (shutdown_)
            throw std::runtime_error("Queue is shutdown");

        buffer_[tail_] = std::forward<U>(item);
        tail_ = (tail_ + 1) % max_size_;
        ++count_;

        lock.unlock();
        cond_not_empty_.notify_one();
    }

    // helper to emplace with timeout
    template<typename Dur, typename U>
    bool try_emplace_for(const Dur& dur, U&& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_not_full_.wait_for(lock, dur, [this]() {
                return count_ < max_size_ || shutdown_;
            }))
            return false;
        if (shutdown_)
            return false;

        buffer_[tail_] = std::forward<U>(item);
        tail_ = (tail_ + 1) % max_size_;
        ++count_;

        lock.unlock();
        cond_not_empty_.notify_one();
        return true;
    }
};