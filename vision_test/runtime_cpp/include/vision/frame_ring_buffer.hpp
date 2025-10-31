#pragma once

#include "common/include/common/types.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace fsai::vision {

class FrameRingBuffer {
 public:
  struct FrameHandle {
    FsaiStereoFrame frame{};
    std::vector<uint8_t> left_storage;
    std::vector<uint8_t> right_storage;
  };

  virtual ~FrameRingBuffer() = default;

  virtual bool tryPush(const FsaiStereoFrame& frame) = 0;
  virtual void push(const FsaiStereoFrame& frame) = 0;

  virtual std::optional<FrameHandle> tryPop() = 0;
  virtual FrameHandle pop() = 0;

  virtual std::size_t capacity() const = 0;
  virtual std::size_t size() const = 0;
};

void copyStereoFrame(const FsaiStereoFrame& source,
                     FrameRingBuffer::FrameHandle& destination);
FrameRingBuffer::FrameHandle cloneStereoFrame(const FsaiStereoFrame& source);
std::unique_ptr<FrameRingBuffer> makeFrameRingBuffer(std::size_t capacity);

}  // namespace fsai::vision

