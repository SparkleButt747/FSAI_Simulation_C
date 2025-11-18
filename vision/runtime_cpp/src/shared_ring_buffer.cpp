#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <stdexcept>
#include <utility> // for std::move

namespace fsai::common {

template <typename T>
class GenericRingBuffer {
public:
    // Constructor: Pre-allocates memory to avoid runtime allocations
    explicit GenericRingBuffer(std::size_t capacity)
        : slots_(capacity), capacity_(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("Buffer capacity must be greater than zero");
        }
    }

    // Delete copy/move constructors for the BUFFER itself to prevent
    // accidental duplication of the synchronization primitives.
    GenericRingBuffer(const GenericRingBuffer&) = delete;
    GenericRingBuffer& operator=(const GenericRingBuffer&) = delete;

    // -------------------------------------------------------
    // PRODUCER METHODS
    // -------------------------------------------------------

    // Non-blocking push. Returns false if buffer is full.
    bool tryPush(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (count_ == capacity_) {
            return false;
        }
        writeUnlocked(item);
        lock.unlock();
        not_empty_.notify_one();
        return true;
    }

    // Blocking push. Waits until there is space.
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this]() { return count_ < capacity_; });
        writeUnlocked(item);
        lock.unlock();
        not_empty_.notify_one();
    }

    // -------------------------------------------------------
    // CONSUMER METHODS
    // -------------------------------------------------------

    // Non-blocking pop. Returns std::nullopt if empty.
    std::optional<T> tryPop() {
        std::unique_lock<std::mutex> lock(mutex_);
        if (count_ == 0) {
            return std::nullopt;
        }
        T item = readUnlocked();
        lock.unlock();
        not_full_.notify_one();
        return item;
    }

    // Blocking pop. Waits until there is data.
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this]() { return count_ > 0; });
        T item = readUnlocked();
        lock.unlock();
        not_full_.notify_one();
        return item;
    }

    // -------------------------------------------------------
    // DIAGNOSTICS
    // -------------------------------------------------------

    std::size_t capacity() const {
        return capacity_;
    }

    std::size_t size() const {
        std::scoped_lock<std::mutex> lock(mutex_);
        return count_;
    }

    bool isEmpty() const {
        std::scoped_lock<std::mutex> lock(mutex_);
        return count_ == 0;
    }

    bool isFull() const {
        std::scoped_lock<std::mutex> lock(mutex_);
        return count_ == capacity_;
    }

private:
    // Helper: Assumes lock is already held
    void writeUnlocked(const T& item) {
        slots_[head_] = item; // Uses T::operator=
        head_ = (head_ + 1) % capacity_;
        ++count_;
    }

    // Helper: Assumes lock is already held
    T readUnlocked() {
        // Use std::move to efficiently transfer ownership out of the buffer
        T item = std::move(slots_[tail_]);

        // Optional: Reset the slot if T holds resources to free them immediately
        // slots_[tail_] = T{};

        tail_ = (tail_ + 1) % capacity_;
        --count_;
        return item;
    }

    // Member Variables
    std::vector<T> slots_;
    const std::size_t capacity_;
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t count_ = 0;

    // Synchronization
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
};

} // namespace fsai::common