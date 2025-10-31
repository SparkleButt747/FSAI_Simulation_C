#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "common/include/common/types.h"

namespace fsai::vision {

class FrameRingBuffer {
 public:
  struct FrameHandle {
    FsaiStereoFrame frame{};
    std::vector<uint8_t> left_storage{};
    std::vector<uint8_t> right_storage{};

    FrameHandle() = default;
    FrameHandle(FrameHandle&&) noexcept = default;
    FrameHandle& operator=(FrameHandle&&) noexcept = default;

    FrameHandle(const FrameHandle&) = delete;
    FrameHandle& operator=(const FrameHandle&) = delete;

    const FsaiStereoFrame& view() const { return frame; }
    FsaiStereoFrame& view() { return frame; }

    void clear() {
      left_storage.clear();
      right_storage.clear();
      frame = FsaiStereoFrame{};
    }
  };

  virtual ~FrameRingBuffer() = default;

  virtual bool tryPush(const FsaiStereoFrame& frame) = 0;
  virtual void push(const FsaiStereoFrame& frame) = 0;

  virtual std::optional<FrameHandle> tryPop() = 0;
  virtual FrameHandle pop() = 0;

  virtual std::size_t capacity() const = 0;
  virtual std::size_t size() const = 0;
};

std::unique_ptr<FrameRingBuffer> makeFrameRingBuffer(std::size_t capacity);

void copyStereoFrame(const FsaiStereoFrame& source, FrameRingBuffer::FrameHandle& destination);
FrameRingBuffer::FrameHandle cloneStereoFrame(const FsaiStereoFrame& source);

}  // namespace fsai::vision

