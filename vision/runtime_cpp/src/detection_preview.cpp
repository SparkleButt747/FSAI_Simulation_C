#include "vision/detection_preview.hpp"
#include "vision/vision_node.hpp"
#include "common/include/common/types.h"

#include <chrono>
#include <utility>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>

namespace fsai {
namespace vision {

namespace {
constexpr std::chrono::milliseconds kIdleSleep{5};
constexpr std::chrono::milliseconds kBufferMissingSleep{50};

std::shared_ptr<SDL_Texture> makeTexture(SDL_Renderer* renderer, int width, int height) {
  SDL_Texture* raw = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32,
                                       SDL_TEXTUREACCESS_STREAMING, width, height);
  if (!raw) {
    return nullptr;
  }
  if (SDL_SetTextureBlendMode(raw, SDL_BLENDMODE_BLEND) != 0) {
    SDL_DestroyTexture(raw);
    return nullptr;
  }
  return std::shared_ptr<SDL_Texture>(raw, [](SDL_Texture* texture) {
    if (texture) {
      SDL_DestroyTexture(texture);
    }
  });
}

cv::Scalar getConeColor(fsai::types::ConeSide side) {
  switch (side) {
    case FSAI_CONE_LEFT: return cv::Scalar(255, 0, 0);    // Blue (BGR)
    case FSAI_CONE_RIGHT: return cv::Scalar(0,255,255); // Yellow (BGR)
    case FSAI_CONE_UNKNOWN:
    default: return cv::Scalar(0, 165, 255);              // Orange (BGR)
  }
}
}  // namespace

DetectionPreview::DetectionPreview() = default;
DetectionPreview::~DetectionPreview() { stop(); }

bool DetectionPreview::start(SDL_Renderer* renderer,
                             std::shared_ptr<VisionNode> node,
                             std::string& error_message) {
  error_message.clear();

  if (running_.load()) {
    return true;
  }
  if (!renderer) {
    error_message = "DetectionPreview: Invalid SDL renderer";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }
  if (!node) {
    error_message = "DetectionPreview: VisionNode is null";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }

  SDL_RendererInfo info{};
  if (SDL_GetRendererInfo(renderer, &info) != 0) {
    error_message = SDL_GetError();
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }
  if ((info.flags & SDL_RENDERER_ACCELERATED) == 0u) {
    error_message = "SDL renderer lacks GPU acceleration";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }

  renderer_ = renderer;
  vision_node_ = node; // Store the node
  {
    std::lock_guard<std::mutex> guard(mutex_);
    texture_.reset();
    width_ = 0;
    height_ = 0;
    last_frame_timestamp_ns_ = 0;
    last_error_.clear();
    worker_buffer_.clear();
    pending_buffer_.clear();
    pending_ready_ = false;
    pending_width_ = 0;
    pending_height_ = 0;
    pending_timestamp_ns_ = 0;
  }

  running_.store(true);
  try {
    worker_ = std::thread(&DetectionPreview::workerLoop, this);
  } catch (...) {
    running_.store(false);
    error_message = "DetectionPreview: Failed to start worker thread";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }

  return true;
}

void DetectionPreview::stop() {
  const bool was_running = running_.exchange(false);
  if (was_running && worker_.joinable()) {
    worker_.join();
  }
  vision_node_.reset();
  renderer_ = nullptr;
  std::lock_guard<std::mutex> lock(mutex_);
  texture_.reset();
  width_ = 0;
  height_ = 0;
  last_frame_timestamp_ns_ = 0;
  worker_buffer_.clear();
  pending_buffer_.clear();
  pending_ready_ = false;
  pending_width_ = 0;
  pending_height_ = 0;
  pending_timestamp_ns_ = 0;
}

bool DetectionPreview::running() const { return running_.load(); }

std::string DetectionPreview::statusMessage() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

bool DetectionPreview::ensureTextureLocked(int width, int height) {
  if (texture_ && width_ == width && height_ == height) {
    return true;
  }
  texture_.reset();
  width_ = 0;
  height_ = 0;
  auto texture = makeTexture(renderer_, width, height);
  if (!texture) {
    last_error_ = SDL_GetError();
    return false;
  }
  texture_ = std::move(texture);
  width_ = width;
  height_ = height;
  return true;
}

DetectionPreview::Snapshot DetectionPreview::snapshot() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_ready_) {
    if (!ensureTextureLocked(pending_width_, pending_height_)) {
      pending_ready_ = false;
    } else if (SDL_UpdateTexture(texture_.get(), nullptr, pending_buffer_.data(),
                                 pending_width_ * 4) != 0) {
      last_error_ = SDL_GetError();
      pending_ready_ = false;
    } else {
      last_error_.clear();
      width_ = pending_width_;
      height_ = pending_height_;
      last_frame_timestamp_ns_ = pending_timestamp_ns_;
      pending_ready_ = false;
    }
  }
  Snapshot snapshot;
  snapshot.texture = texture_;
  snapshot.width = width_;
  snapshot.height = height_;
  snapshot.frame_timestamp_ns = last_frame_timestamp_ns_;
  return snapshot;
}

void DetectionPreview::workerLoop() {
  while (running_.load()) {
    auto vision_node = vision_node_;
    if (!vision_node) {
      std::this_thread::sleep_for(kBufferMissingSleep);
      continue;
    }

    // 1. Atomic grab of all data
    RenderableFrame data = vision_node->getRenderableFrame();

    // FsaiDetections cone_data = vision_node->makeDetections();

    // 2. Check if we have valid data and if it's new
    if (!data.valid || data.image.empty()) {
       std::this_thread::sleep_for(kIdleSleep);
       continue;
    }

    // (Optional optimization: skip if we've already rendered this timestamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data.timestamp_ns <= last_frame_timestamp_ns_ && !pending_ready_) {
             // We already rendered this frame, and nothing is pending.
             std::this_thread::sleep_for(kIdleSleep);
             continue;
        }
    }

    // 4. Draw Boxes
    for (const auto& box : data.boxes) {
      cv::Scalar color = getConeColor(box.side);
      cv::Point p1(box.x, box.y);
      cv::Point p2(box.x + static_cast<int>(box.w), box.y + static_cast<int>(box.h));
      cv::rectangle(data.image, p1, p2, color, 2);

      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << box.conf;
      cv::Point text_pos(box.x, box.y - 5);
      cv::putText(data.image, ss.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
                  0.5, color, 1, cv::LINE_AA);
    }

    // 5. Convert to RGBA for SDL
    const std::size_t pixel_count =
        static_cast<std::size_t>(data.image.cols) * static_cast<std::size_t>(data.image.rows);
    worker_buffer_.resize(pixel_count * 4u);
    // Note: Use bgr_image.rows/cols, not the old 'left' struct
    cv::Mat rgba_image(data.image.rows, data.image.cols, CV_8UC4, worker_buffer_.data(),
                       static_cast<size_t>(data.image.cols) * 4u);
    cv::cvtColor(data.image, rgba_image, cv::COLOR_BGR2RGBA);

    // 6. Publish to main thread
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_buffer_.swap(worker_buffer_);
        pending_width_ = rgba_image.cols;
        pending_height_ = rgba_image.rows;
        pending_timestamp_ns_ = data.timestamp_ns;
        pending_ready_ = true; 
    }
  }
}

}  // namespace vision
}  // namespace fsai