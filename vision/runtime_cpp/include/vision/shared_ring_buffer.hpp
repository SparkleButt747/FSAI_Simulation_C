#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <vector>
#include <stdexcept>

namespace fsai::vision {

template <typename T>
class GenericRingBuffer{
    public:
    /**
     * @param capacity The size of the ring buffer
     */
    explicit GenericRingBuffer(std::size_t capacity);
    /**
     * @param T
     * @return
     */
    bool tryPush(const T& item);
    /**
     * @param T item to be pushed onto the ring buffer
     * @returns false if buffer is at capacity, false otherwise
     */
    void push(const T& item);
    /**
     * @returns T or if buffer is empty std::nullopt
     */
    std::optional<T> tryPop();
    /**
     * @returns T, most recent item from buffer
     * @details blocking pop, will wait for item
     */
    T pop();
    std::size_t capacity() const;
    std::size_t size() const;
    bool isEmpty() const;
    bool isFull() const;
    private:
    void writeUnlocked(const T& item);
    T readUnlocked();
    std::vector<T> slots_;
    const std::size_t capacity_;
    std::size_t head_ = 0;

};

} // namespace fsai::common