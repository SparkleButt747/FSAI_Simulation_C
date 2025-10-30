#pragma once

#include <cstdint>

#include <SDL.h>

#include <common/types.h>

namespace fsai::sim::integration {

class StereoDisplay {
 public:
  StereoDisplay() = default;
  ~StereoDisplay();

  StereoDisplay(const StereoDisplay&) = delete;
  StereoDisplay& operator=(const StereoDisplay&) = delete;

  void present(const FsaiStereoFrame& frame);

 private:
  bool ensureResources(const FsaiStereoFrame& frame);
  void reset();

  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Texture* left_texture_ = nullptr;
  SDL_Texture* right_texture_ = nullptr;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace fsai::sim::integration
