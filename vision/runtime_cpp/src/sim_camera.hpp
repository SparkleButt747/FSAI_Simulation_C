#ifndef FSAI_VISION_SIM_CAMERA_HPP
#define FSAI_VISION_SIM_CAMERA_HPP

#include "vision/frame_ring_buffer.hpp"
#include <memory>
#include <optional>

namespace fsai::vision {

/**
 * @class SimCamera
 * @brief Handles frame retrieval from the active FrameRingBuffer.
 *
 * This class connects to the globally active FrameRingBuffer and provides
 * methods to retrieve the latest stereo frame. It's designed to "latch"
 * to the most recent frame, discarding any stale frames in the queue
 * to minimize processing latency.
 */
class SimCamera {
 public:
  /**
   * @brief Constructs a SimCamera and connects to the active FrameRingBuffer.
   * @throws std::runtime_error if no active FrameRingBuffer is set.
   */
  SimCamera();

  /**
   * @brief Default destructor.
   */
  ~SimCamera() = default;

  // Delete copy/move constructors and assignment operators for simplicity
  SimCamera(const SimCamera&) = delete;
  SimCamera& operator=(const SimCamera&) = delete;
  SimCamera(SimCamera&&) = delete;
  SimCamera& operator=(SimCamera&&) = delete;

  /**
   * @brief Blocks until a new frame is available, then returns the most recent one.
   *
   * This method will wait for at least one frame using `pop()`, then
   * immediately drain any other frames in the queue using `tryPop()`
   * until the queue is empty. It then returns the last frame it
   * retrieved, ensuring it's the most recent one.
   *
   * @return A FrameHandle owning the most recent stereo frame data.
   */
  FrameRingBuffer::FrameHandle getLatestFrame();

  /**
   * @brief Tries to get the most recent frame without blocking.
   *
   * This method drains the queue using `tryPop()` and returns the
   * most recent frame. If the queue was empty, it returns std::nullopt.
   *
   * @return An optional FrameHandle; contains the frame if available,
   * otherwise std::nullopt.
   */
  std::optional<FrameRingBuffer::FrameHandle> tryGetLatestFrame();

 private:
  std::shared_ptr<FrameRingBuffer> buffer_;
};

}  // namespace fsai::vision

#endif  // FSAI_VISION_SIM_CAMERA_HPP