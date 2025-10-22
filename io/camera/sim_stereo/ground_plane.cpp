#include "ground_plane.hpp"

#include <utility>

#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {

namespace {
GroundVertex makeGroundVertex(float x, float y, float z, float r, float g, float b) {
  return GroundVertex{{x, y, z}, {r, g, b}};
}
}  // namespace

GroundPlane::GroundPlane() {
  vertices_.reserve(4);
  indices_.reserve(6);

  constexpr float kExtent = 300.0f;
  vertices_.push_back(
      makeGroundVertex(-kExtent, 0.0f, -kExtent, 0.25f, 0.25f, 0.25f));
  vertices_.push_back(
      makeGroundVertex(kExtent, 0.0f, -kExtent, 0.3f, 0.3f, 0.3f));
  vertices_.push_back(makeGroundVertex(kExtent, 0.0f, kExtent, 0.35f, 0.35f, 0.35f));
  vertices_.push_back(
      makeGroundVertex(-kExtent, 0.0f, kExtent, 0.3f, 0.3f, 0.3f));

  indices_.insert(indices_.end(), {0, 1, 2, 0, 2, 3});
}

GroundPlane::GroundPlane(GroundPlane&& other) noexcept { *this = std::move(other); }

GroundPlane& GroundPlane::operator=(GroundPlane&& other) noexcept {
  if (this != &other) {
    destroyGl();
    vertices_ = std::move(other.vertices_);
    indices_ = std::move(other.indices_);
    vao_ = other.vao_;
    vbo_ = other.vbo_;
    ebo_ = other.ebo_;
    other.vao_ = other.vbo_ = other.ebo_ = 0;
  }
  return *this;
}

GroundPlane::~GroundPlane() { destroyGl(); }

void GroundPlane::destroyGl() {
  if (ebo_ != 0) {
    glDeleteBuffers(1, &ebo_);
    ebo_ = 0;
  }
  if (vbo_ != 0) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
  if (vao_ != 0) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
}

void GroundPlane::initializeGl() {
  if (vao_ != 0) {
    return;
  }
  uploadGeometry();
}

void GroundPlane::uploadGeometry() {
  glGenVertexArrays(1, &vao_);
  glBindVertexArray(vao_);

  glGenBuffers(1, &vbo_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(vertices_.size() * sizeof(GroundVertex)),
               vertices_.data(), GL_STATIC_DRAW);

  glGenBuffers(1, &ebo_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(indices_.size() * sizeof(uint32_t)),
               indices_.data(), GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GroundVertex),
                        reinterpret_cast<void*>(offsetof(GroundVertex, position)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GroundVertex),
                        reinterpret_cast<void*>(offsetof(GroundVertex, color)));

  glBindVertexArray(0);
}

void GroundPlane::draw() const {
  if (vao_ == 0) {
    return;
  }
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()),
                 GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

}  // namespace fsai::io::camera::sim_stereo
