#include "vision/frame_ring_buffer.hpp"

#include <condition_variable>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fsai::vision {
namespace {
std::size_t frameByteCount(const FsaiFrame& frame) {
  if (frame.h <= 0 || frame.stride <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(frame.stride) * static_cast<std::size_t>(frame.h);
}

void copyFrame(const FsaiFrame& source, FsaiFrame& destination,
               std::vector<uint8_t>& storage) {
  destination = source;
  const std::size_t bytes = frameByteCount(source);
  storage.resize(bytes);
  if (bytes > 0 && source.data != nullptr) {
    std::memcpy(storage.data(), source.data, bytes);
    destination.data = storage.data();
  } else {
    destination.data = nullptr;
  }
}

class DefaultFrameRingBuffer : public FrameRingBuffer {
 public:
  explicit DefaultFrameRingBuffer(std::size_t capacity)
      : slots_(capacity), capacity_(capacity) {
    if (capacity == 0) {
      throw std::invalid_argument("FrameRingBuffer capacity must be greater than zero");
    }
  }

  bool tryPush(const FsaiStereoFrame& frame) override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (count_ == capacity_) {
      return false;
    }
    writeUnlocked(frame);
    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  void push(const FsaiStereoFrame& frame) override {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [this]() { return count_ < capacity_; });
    writeUnlocked(frame);
    lock.unlock();
    not_empty_.notify_one();
  }

  std::optional<FrameHandle> tryPop() override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (count_ == 0) {
      return std::nullopt;
    }
    FrameHandle handle = readUnlocked();
    lock.unlock();
    not_full_.notify_one();
    return handle;
  }

  FrameHandle pop() override {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this]() { return count_ > 0; });
    FrameHandle handle = readUnlocked();
    lock.unlock();
    not_full_.notify_one();
    return handle;
  }

  std::size_t capacity() const override { return capacity_; }

  std::size_t size() const override {
    std::scoped_lock<std::mutex> lock(mutex_);
    return count_;
  }

 private:
  void writeUnlocked(const FsaiStereoFrame& frame) {
    copyStereoFrame(frame, slots_[head_]);
    head_ = (head_ + 1) % capacity_;
    ++count_;
  }

  FrameHandle readUnlocked() {
    FrameHandle handle = std::move(slots_[tail_]);
    slots_[tail_].frame = FsaiStereoFrame{};
    tail_ = (tail_ + 1) % capacity_;
    --count_;
    return handle;
  }

  std::vector<FrameHandle> slots_;
  const std::size_t capacity_;
  std::size_t head_ = 0;
  std::size_t tail_ = 0;
  std::size_t count_ = 0;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
};

}  // namespace

void copyStereoFrame(const FsaiStereoFrame& source,
                     FrameRingBuffer::FrameHandle& destination) {
  destination.frame.t_sync_ns = source.t_sync_ns;
  copyFrame(source.left, destination.frame.left, destination.left_storage);
  copyFrame(source.right, destination.frame.right, destination.right_storage);
}

FrameRingBuffer::FrameHandle cloneStereoFrame(const FsaiStereoFrame& source) {
  FrameRingBuffer::FrameHandle handle{};
  copyStereoFrame(source, handle);
  return handle;
}

std::unique_ptr<FrameRingBuffer> makeFrameRingBuffer(std::size_t capacity) {
  return std::make_unique<DefaultFrameRingBuffer>(capacity);
}

}  // namespace fsai::vision

