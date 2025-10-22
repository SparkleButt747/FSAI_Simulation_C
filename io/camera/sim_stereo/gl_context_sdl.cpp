#include "io/camera/sim_stereo/gl_context_sdl.hpp"

#include <stdexcept>
#include <utility>

namespace fsai::io::camera::sim_stereo {

namespace {
void setGlAttributes() {
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
}
}  // namespace

GlContextSdl::GlContextSdl() {
  if (SDL_WasInit(SDL_INIT_VIDEO) == 0) {
    if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
      throw std::runtime_error(std::string{"SDL video init failed: "} +
                               SDL_GetError());
    }
  }

  setGlAttributes();

  window_ = SDL_CreateWindow("SimStereoOffscreen", SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, 16, 16,
                             SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  if (!window_) {
    throw std::runtime_error(std::string{"SDL_CreateWindow failed: "} +
                             SDL_GetError());
  }

  context_ = SDL_GL_CreateContext(window_);
  if (!context_) {
    throw std::runtime_error(std::string{"SDL_GL_CreateContext failed: "} +
                             SDL_GetError());
  }

  SDL_GL_MakeCurrent(window_, context_);
  SDL_GL_SetSwapInterval(0);
}

GlContextSdl::GlContextSdl(GlContextSdl&& other) noexcept {
  std::swap(window_, other.window_);
  std::swap(context_, other.context_);
}

GlContextSdl& GlContextSdl::operator=(GlContextSdl&& other) noexcept {
  if (this != &other) {
    destroy();
    std::swap(window_, other.window_);
    std::swap(context_, other.context_);
  }
  return *this;
}

GlContextSdl::~GlContextSdl() { destroy(); }

void GlContextSdl::destroy() {
  if (context_) {
    SDL_GL_DeleteContext(context_);
    context_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
}

void GlContextSdl::makeCurrent() {
  if (window_ && context_) {
    SDL_GL_MakeCurrent(window_, context_);
  }
}

void GlContextSdl::clearCurrent() { SDL_GL_MakeCurrent(nullptr, nullptr); }

void GlContextSdl::swap() {
  if (window_) {
    SDL_GL_SwapWindow(window_);
  }
}

}  // namespace fsai::io::camera::sim_stereo
