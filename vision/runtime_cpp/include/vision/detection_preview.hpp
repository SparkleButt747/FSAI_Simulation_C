#pragma once

#include "common/include/common/types.h"
#include "detect.hpp"
#include "frame_ring_buffer.hpp"
#include "vision_node.hpp" // <-- 1. ADD THIS

#include <SDL.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief This largely follows the logic implemented in vision_test/runtime_cpp/include/vision/edge_preview.hpp
 * and provides rendering of the box bounds to show the vision pipeline working
 */

namespace fsai {
namespace vision {

class DetectionPreview {
 public:
  struct Snapshot {
    std::shared_ptr<SDL_Texture> texture; // <-- 2. FIX: Typo SDL_TEXTURE
    int width = 0;
    int height = 0;
    uint64_t frame_timestamp_ns = 0;
  };
  DetectionPreview();
  ~DetectionPreview();

  // 3. FIX: Add 'std::shared_ptr<VisionNode> node' parameter
  bool start(SDL_Renderer* renderer,
             std::shared_ptr<VisionNode> node, std::string& error_message);
  void stop();

  [[nodiscard]] bool running() const;
  [[nodiscard]] Snapshot snapshot();
  [[nodiscard]] std::string statusMessage() const;

 private:
  void workerLoop();
  bool ensureTextureLocked(int width, int height);

  SDL_Renderer* renderer_ = nullptr;
  std::weak_ptr<FrameRingBuffer> buffer_;
  std::shared_ptr<VisionNode> vision_node_; 
  std::thread worker_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  std::shared_ptr<SDL_Texture> texture_;
  int width_ = 0;
  int height_ = 0;
  uint64_t last_frame_timestamp_ns_ = 0;
  std::string last_error_;
  std::vector<uint8_t> worker_buffer_;
  std::vector<uint8_t> pending_buffer_;
  bool pending_ready_ = false;
  int pending_width_ = 0;
  int pending_height_ = 0;
  uint64_t pending_timestamp_ns_ = 0;
};
}  // namespace vision
}  // namespace fsai