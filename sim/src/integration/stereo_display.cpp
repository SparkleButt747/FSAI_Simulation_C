#include "stereo_display.hpp"

#include <cstdio>

namespace fsai::sim::integration {

StereoDisplay::~StereoDisplay() { reset(); }

bool StereoDisplay::ensureResources(const FsaiStereoFrame& frame) {
  if (window_ && frame.left.w == width_ && frame.left.h == height_) {
    return true;
  }

  reset();

  width_ = frame.left.w;
  height_ = frame.left.h;

  window_ = SDL_CreateWindow("FS-AI Stereo Simulation",
                             SDL_WINDOWPOS_CENTERED,
                             SDL_WINDOWPOS_CENTERED,
                             width_ * 2,
                             height_,
                             SDL_WINDOW_SHOWN);
  if (!window_) {
    std::fprintf(stderr, "Failed to create stereo window: %s\n", SDL_GetError());
    reset();
    return false;
  }

  renderer_ = SDL_CreateRenderer(window_, -1,
                                 SDL_RENDERER_ACCELERATED |
                                     SDL_RENDERER_PRESENTVSYNC);
  if (!renderer_) {
    std::fprintf(stderr, "Failed to create stereo renderer: %s\n", SDL_GetError());
    reset();
    return false;
  }

  left_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24,
                                    SDL_TEXTUREACCESS_STREAMING,
                                    width_, height_);
  right_texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24,
                                     SDL_TEXTUREACCESS_STREAMING,
                                     width_, height_);
  if (!left_texture_ || !right_texture_) {
    std::fprintf(stderr, "Failed to create stereo textures: %s\n", SDL_GetError());
    reset();
    return false;
  }

  return true;
}

void StereoDisplay::present(const FsaiStereoFrame& frame) {
  if (!ensureResources(frame)) {
    return;
  }

  SDL_UpdateTexture(left_texture_, nullptr, frame.left.data, frame.left.stride);
  SDL_UpdateTexture(right_texture_, nullptr, frame.right.data, frame.right.stride);

  SDL_RenderClear(renderer_);

  SDL_Rect left_rect{0, 0, width_, height_};
  SDL_Rect right_rect{width_, 0, width_, height_};
  SDL_RenderCopy(renderer_, left_texture_, nullptr, &left_rect);
  SDL_RenderCopy(renderer_, right_texture_, nullptr, &right_rect);
  SDL_RenderPresent(renderer_);
}

void StereoDisplay::reset() {
  if (left_texture_) {
    SDL_DestroyTexture(left_texture_);
    left_texture_ = nullptr;
  }
  if (right_texture_) {
    SDL_DestroyTexture(right_texture_);
    right_texture_ = nullptr;
  }
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  width_ = 0;
  height_ = 0;
}

}  // namespace fsai::sim::integration
