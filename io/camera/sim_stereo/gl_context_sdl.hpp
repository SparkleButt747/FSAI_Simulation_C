#pragma once

#include <memory>
#include <string>

#include <SDL.h>

namespace fsai::io::camera::sim_stereo {

// Simple RAII wrapper around an off-screen SDL2 OpenGL context.
class GlContextSdl {
 public:
  GlContextSdl();
  GlContextSdl(const GlContextSdl&) = delete;
  GlContextSdl& operator=(const GlContextSdl&) = delete;
  GlContextSdl(GlContextSdl&&) noexcept;
  GlContextSdl& operator=(GlContextSdl&&) noexcept;
  ~GlContextSdl();

  // Make the OpenGL context current on the calling thread.
  void makeCurrent();
  // Clear the context binding on the calling thread.
  void clearCurrent();
  // Swap the hidden window buffers; useful when rendering into the default FBO.
  void swap();

  bool valid() const { return window_ != nullptr && context_ != nullptr; }

 private:
  void destroy();

  SDL_Window* window_ = nullptr;
  SDL_GLContext context_ = nullptr;
};

}  // namespace fsai::io::camera::sim_stereo
