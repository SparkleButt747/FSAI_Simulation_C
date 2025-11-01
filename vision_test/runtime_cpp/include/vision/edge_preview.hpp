#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <SDL.h>

#include "vision/frame_ring_buffer.hpp"

#ifndef FSAI_EDGE_PREVIEW_HAS_OPENCV
#  if defined(__has_include)
#    if __has_include(<opencv2/imgproc.hpp>)
#      define FSAI_EDGE_PREVIEW_HAS_OPENCV 1
#    else
#      define FSAI_EDGE_PREVIEW_HAS_OPENCV 0
#    endif
#  else
#    define FSAI_EDGE_PREVIEW_HAS_OPENCV 1
#  endif
#endif

namespace fsai::vision {

class EdgePreview {
 public:
  struct Snapshot {
    std::shared_ptr<SDL_Texture> texture;
    int width = 0;
    int height = 0;
    uint64_t frame_timestamp_ns = 0;
  };

  EdgePreview();
  ~EdgePreview();

  EdgePreview(const EdgePreview&) = delete;
  EdgePreview& operator=(const EdgePreview&) = delete;

  bool start(SDL_Renderer* renderer, std::shared_ptr<FrameRingBuffer> buffer,
             std::string& error_message);
  void stop();

  [[nodiscard]] bool running() const;
  [[nodiscard]] Snapshot snapshot() const;
  [[nodiscard]] std::string statusMessage() const;

 private:
  void workerLoop();
  bool ensureTextureLocked(int width, int height);

  SDL_Renderer* renderer_ = nullptr;
  std::weak_ptr<FrameRingBuffer> buffer_;
  std::thread worker_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  std::shared_ptr<SDL_Texture> texture_;
  int width_ = 0;
  int height_ = 0;
  uint64_t last_frame_timestamp_ns_ = 0;
  std::string last_error_;
  std::vector<uint8_t> upload_buffer_;
};

}  // namespace fsai::vision

