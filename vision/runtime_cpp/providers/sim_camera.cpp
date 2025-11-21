#include "../src/sim_camera.hpp"

#include <stdexcept>  // For std::runtime_error
#include <utility>    // For std::move

namespace fsai::vision {

SimCamera::SimCamera() : buffer_(getActiveFrameRingBuffer()) {
  if (!buffer_) {
    // If the simulation hasn't set the active buffer yet, this is a
    // critical configuration error.
    throw std::runtime_error("VisionNode: SimCamera: Active FrameRingBuffer is not set or is null.");
  }
}

FrameRingBuffer::FrameHandle SimCamera::getLatestFrame() {
  // 1. Block and wait for the *first* frame to arrive.
  //    This ensures we don't return if the buffer is empty.
  FrameRingBuffer::FrameHandle latest_handle = buffer_->pop();

  // 2. Drain the queue.
  //    Keep trying to pop frames non-blockingly. As long as we get a
  //    new frame, discard the old one and keep the new one.
  while (true) {
    std::optional<FrameRingBuffer::FrameHandle> next_handle = buffer_->tryPop();
    if (!next_handle) {
      // The queue is now empty. `latest_handle` is the most recent.
      return latest_handle;
    }
    // A newer frame was found. Move it into our handle.
    latest_handle = std::move(*next_handle);
  }
}

std::optional<FrameRingBuffer::FrameHandle> SimCamera::tryGetLatestFrame() {
  // 1. Try to get a frame.
  std::optional<FrameRingBuffer::FrameHandle> latest_handle = buffer_->tryPop();

  if (!latest_handle) {
    // The queue was empty. Return immediately.
    return std::nullopt;
  }

  // 2. Drain the queue.
  //    Since we know we have at least one frame, we can loop
  //    and replace it with any newer frames we find.
  while (true) {
    std::optional<FrameRingBuffer::FrameHandle> next_handle = buffer_->tryPop();
    if (!next_handle) {
      // The queue is empty. Return the last frame we found.
      return latest_handle;
    }
    // A newer frame was found. Move it into our handle.
    latest_handle = std::move(*next_handle);
  }
}

}  // namespace fsai::vision