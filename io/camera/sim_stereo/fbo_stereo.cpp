#include "fbo_stereo.hpp"

#include <stdexcept>
#include <utility>

#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {

FboStereo::FboStereo() = default;

FboStereo::FboStereo(FboStereo&& other) noexcept { *this = std::move(other); }

FboStereo& FboStereo::operator=(FboStereo&& other) noexcept {
  if (this != &other) {
    destroyEye(left_);
    destroyEye(right_);
    left_ = other.left_;
    right_ = other.right_;
    width_ = other.width_;
    height_ = other.height_;
    other.left_ = {};
    other.right_ = {};
    other.width_ = other.height_ = 0;
  }
  return *this;
}

FboStereo::~FboStereo() {
  destroyEye(left_);
  destroyEye(right_);
}

void FboStereo::destroyEye(EyeTarget& eye) {
  if (eye.depth != 0) {
    glDeleteRenderbuffers(1, &eye.depth);
    eye.depth = 0;
  }
  if (eye.color != 0) {
    glDeleteTextures(1, &eye.color);
    eye.color = 0;
  }
  if (eye.fbo != 0) {
    glDeleteFramebuffers(1, &eye.fbo);
    eye.fbo = 0;
  }
}

void FboStereo::ensureEye(EyeTarget& eye) {
  if (eye.fbo != 0 && eye.color != 0 && eye.depth != 0) {
    return;
  }

  glGenFramebuffers(1, &eye.fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, eye.fbo);

  glGenTextures(1, &eye.color);
  glBindTexture(GL_TEXTURE_2D, eye.color);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         eye.color, 0);

  glGenRenderbuffers(1, &eye.depth);
  glBindRenderbuffer(GL_RENDERBUFFER, eye.depth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER, eye.depth);

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error("Failed to create stereo framebuffer");
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FboStereo::resize(int width, int height) {
  if (width == width_ && height == height_) {
    return;
  }
  width_ = width;
  height_ = height;

  destroyEye(left_);
  destroyEye(right_);

  if (width_ > 0 && height_ > 0) {
    ensureEye(left_);
    ensureEye(right_);
  }
}

void FboStereo::renderEye(EyeTarget& eye, const std::array<float, 16>& view,
                          const std::array<float, 16>& proj,
                          ShaderProgram& cone_shader,
                          ShaderProgram& ground_shader, const ConeMesh& cones,
                          const GroundPlane& ground) {
  glBindFramebuffer(GL_FRAMEBUFFER, eye.fbo);
  glViewport(0, 0, width_, height_);
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.55f, 0.65f, 0.75f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  ground_shader.use();
  ground_shader.setMat4("u_view", view);
  ground_shader.setMat4("u_proj", proj);
  ground.draw();

  cone_shader.use();
  cone_shader.setMat4("u_view", view);
  cone_shader.setMat4("u_proj", proj);
  cones.draw();
}

void FboStereo::renderScene(const ConeMesh& cones, const GroundPlane& ground,
                            ShaderProgram& cone_shader,
                            ShaderProgram& ground_shader,
                            const std::array<float, 16>& view_left,
                            const std::array<float, 16>& proj_left,
                            const std::array<float, 16>& view_right,
                            const std::array<float, 16>& proj_right) {
  if (width_ <= 0 || height_ <= 0) {
    return;
  }
  ensureEye(left_);
  ensureEye(right_);

  renderEye(left_, view_left, proj_left, cone_shader, ground_shader, cones,
            ground);
  renderEye(right_, view_right, proj_right, cone_shader, ground_shader, cones,
            ground);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace fsai::io::camera::sim_stereo
