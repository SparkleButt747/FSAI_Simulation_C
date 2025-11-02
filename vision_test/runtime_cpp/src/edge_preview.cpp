#include "vision/edge_preview.hpp"

#include <chrono>
#include <utility>

#if FSAI_EDGE_PREVIEW_HAS_OPENCV
#include <opencv2/imgproc.hpp>
#endif

namespace fsai::vision {

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
}  // namespace

EdgePreview::EdgePreview() = default;

EdgePreview::~EdgePreview() { stop(); }

bool EdgePreview::start(SDL_Renderer* renderer,
                        std::shared_ptr<FrameRingBuffer> buffer,
                        std::string& error_message) {
  error_message.clear();

  if (running_.load()) {
    return true;
  }

  if (!renderer) {
    error_message = "Invalid SDL renderer";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }

  if (!buffer) {
    error_message = "Stereo frame buffer unavailable";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }

#if !FSAI_EDGE_PREVIEW_HAS_OPENCV
  error_message = "OpenCV imgproc module not available";
  std::lock_guard<std::mutex> lock(mutex_);
  last_error_ = error_message;
  return false;
#else
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
  buffer_ = buffer;
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
    worker_ = std::thread(&EdgePreview::workerLoop, this);
  } catch (...) {
    running_.store(false);
    error_message = "Failed to start edge preview worker thread";
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error_message;
    return false;
  }

  return true;
#endif
}

void EdgePreview::stop() {
  const bool was_running = running_.exchange(false);
  if (was_running && worker_.joinable()) {
    worker_.join();
  }
  buffer_.reset();
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

bool EdgePreview::running() const { return running_.load(); }

EdgePreview::Snapshot EdgePreview::snapshot() {
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

std::string EdgePreview::statusMessage() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

bool EdgePreview::ensureTextureLocked(int width, int height) {
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

void EdgePreview::workerLoop() {
#if !FSAI_EDGE_PREVIEW_HAS_OPENCV
  return;
#else
  while (running_.load()) {
    auto buffer = buffer_.lock();
    if (!buffer) {
      std::this_thread::sleep_for(kBufferMissingSleep);
      continue;
    }

    auto handle = buffer->tryPop();
    if (!handle.has_value()) {
      std::this_thread::sleep_for(kIdleSleep);
      continue;
    }

    FrameRingBuffer::FrameHandle frame_handle = std::move(*handle);
    while (auto next = buffer->tryPop()) {
      frame_handle = std::move(*next);
    }

    const FsaiFrame& left = frame_handle.frame.left;
    if (!left.data || left.w <= 0 || left.h <= 0 || left.stride <= 0 ||
        left.fmt != FSAI_PIXEL_RGB888) {
      continue;
    }

    cv::Mat left_rgb(left.h, left.w, CV_8UC3, left.data,
                     static_cast<size_t>(left.stride));
    cv::Mat gray;
    cv::cvtColor(left_rgb, gray, cv::COLOR_RGB2GRAY);
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.2);
    cv::Mat grad_x;
    cv::Mat grad_y;
    cv::Sobel(blurred, grad_x, CV_16S, 1, 0, 3);
    cv::Sobel(blurred, grad_y, CV_16S, 0, 1, 3);
    cv::Mat abs_grad_x;
    cv::Mat abs_grad_y;
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::Mat grad;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0.0, grad);
    cv::Mat edges;
    cv::Canny(grad, edges, 50.0, 150.0);

    const std::size_t pixel_count =
        static_cast<std::size_t>(left.w) * static_cast<std::size_t>(left.h);
    worker_buffer_.resize(pixel_count * 4u);
    cv::Mat rgba(left.h, left.w, CV_8UC4, worker_buffer_.data(),
                 static_cast<size_t>(left.w) * 4u);
    cv::cvtColor(edges, rgba, cv::COLOR_GRAY2RGBA);

    const uint64_t timestamp = frame_handle.frame.t_sync_ns != 0
                                   ? frame_handle.frame.t_sync_ns
                                   : left.t_ns;

    std::lock_guard<std::mutex> lock(mutex_);
    pending_buffer_.swap(worker_buffer_);
    pending_width_ = left.w;
    pending_height_ = left.h;
    pending_timestamp_ns_ = timestamp;
    pending_ready_ = true;
  }
#endif
}

}  // namespace fsai::vision

