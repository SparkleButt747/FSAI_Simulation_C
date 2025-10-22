#include "readback_pbo.hpp"

#include <stdexcept>
#include <utility>

#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {

ReadbackPbo::ReadbackPbo() = default;

ReadbackPbo::ReadbackPbo(ReadbackPbo&& other) noexcept { *this = std::move(other); }

ReadbackPbo& ReadbackPbo::operator=(ReadbackPbo&& other) noexcept {
  if (this != &other) {
    if (pbo_ != 0) {
      glDeleteBuffers(1, &pbo_);
    }
    pbo_ = other.pbo_;
    capacity_ = other.capacity_;
    other.pbo_ = 0;
    other.capacity_ = 0;
  }
  return *this;
}

ReadbackPbo::~ReadbackPbo() {
  if (pbo_ != 0) {
    glDeleteBuffers(1, &pbo_);
  }
}

void ReadbackPbo::ensureCapacity(std::size_t bytes) {
  if (pbo_ == 0) {
    glGenBuffers(1, &pbo_);
  }
  if (bytes > capacity_) {
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_);
    glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr,
                 GL_STREAM_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    capacity_ = bytes;
  }
}

void ReadbackPbo::read(unsigned int fbo, int width, int height,
                        std::vector<uint8_t>& out) {
  const std::size_t required = static_cast<std::size_t>(width) *
                               static_cast<std::size_t>(height) * 4u;
  ensureCapacity(required);

  GLint previous_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previous_fbo);

  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

  void* ptr = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                               static_cast<GLsizeiptr>(required), GL_MAP_READ_BIT);
  if (!ptr) {
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, previous_fbo);
    throw std::runtime_error("Failed to map pixel buffer for readback");
  }

  out.assign(static_cast<uint8_t*>(ptr),
             static_cast<uint8_t*>(ptr) + static_cast<std::ptrdiff_t>(required));

  glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, previous_fbo);
}

}  // namespace fsai::io::camera::sim_stereo
