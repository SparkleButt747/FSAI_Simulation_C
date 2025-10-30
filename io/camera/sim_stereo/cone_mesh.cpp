#include <io/camera/sim_stereo/cone_mesh.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {

namespace {
constexpr int kSegments = 24;

ConeVertex makeVertex(float x, float y, float z, float r, float g, float b) {
  return ConeVertex{{x, y, z}, {r, g, b}};
}

}  // namespace

ConeMesh::ConeMesh() {
  vertices_.reserve(kSegments + 2);

  vertices_.push_back(makeVertex(0.0f, 0.5f, 0.0f, 1.0f, 0.6f, 0.1f));

  for (int i = 0; i < kSegments; ++i) {
    float angle = static_cast<float>(i) / static_cast<float>(kSegments) *
                  static_cast<float>(2.0 * M_PI);
    float x = 0.25f * std::cos(angle);
    float z = 0.25f * std::sin(angle);
    vertices_.push_back(makeVertex(x, 0.0f, z, 1.0f, 0.8f, 0.3f));
  }

  for (int i = 0; i < kSegments; ++i) {
    uint32_t next = (i + 1) % kSegments;
    indices_.push_back(0);
    indices_.push_back(1 + next);
    indices_.push_back(1 + i);
  }

  uint32_t base_center = static_cast<uint32_t>(vertices_.size());
  vertices_.push_back(makeVertex(0.0f, 0.0f, 0.0f, 0.3f, 0.3f, 0.3f));
  for (int i = 0; i < kSegments; ++i) {
    uint32_t next = (i + 1) % kSegments;
    indices_.push_back(base_center);
    indices_.push_back(1 + i);
    indices_.push_back(1 + next);
  }
}

ConeMesh::ConeMesh(ConeMesh&& other) noexcept { *this = std::move(other); }

ConeMesh& ConeMesh::operator=(ConeMesh&& other) noexcept {
  if (this != &other) {
    destroyGl();
    vertices_ = std::move(other.vertices_);
    indices_ = std::move(other.indices_);
    instance_mats_ = std::move(other.instance_mats_);
    instance_count_ = other.instance_count_;
    vao_ = other.vao_;
    vbo_ = other.vbo_;
    ebo_ = other.ebo_;
    instance_vbo_ = other.instance_vbo_;
    other.vao_ = other.vbo_ = other.ebo_ = other.instance_vbo_ = 0;
    other.instance_count_ = 0;
  }
  return *this;
}

ConeMesh::~ConeMesh() { destroyGl(); }

void ConeMesh::destroyGl() {
  if (instance_vbo_ != 0) {
    glDeleteBuffers(1, &instance_vbo_);
    instance_vbo_ = 0;
  }
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

void ConeMesh::initializeGl() {
  if (vao_ != 0) {
    return;
  }
  uploadGeometry();
}

void ConeMesh::uploadGeometry() {
  glGenVertexArrays(1, &vao_);
  glBindVertexArray(vao_);

  glGenBuffers(1, &vbo_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(vertices_.size() * sizeof(ConeVertex)),
               vertices_.data(), GL_STATIC_DRAW);

  glGenBuffers(1, &ebo_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(indices_.size() * sizeof(uint32_t)),
               indices_.data(), GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ConeVertex),
                        reinterpret_cast<void*>(offsetof(ConeVertex, position)));

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ConeVertex),
                        reinterpret_cast<void*>(offsetof(ConeVertex, color)));

  glGenBuffers(1, &instance_vbo_);
  glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
  glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

  constexpr std::size_t mat_stride = sizeof(float) * 16;
  for (int i = 0; i < 4; ++i) {
    glEnableVertexAttribArray(2 + i);
    glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE,
                          static_cast<GLsizei>(mat_stride),
                          reinterpret_cast<void*>(sizeof(float) * 4 * i));
    glVertexAttribDivisor(2 + i, 1);
  }

  glBindVertexArray(0);
}

void ConeMesh::setInstances(const std::vector<float>& matrices4x4) {
  instance_mats_ = matrices4x4;
  instance_count_ = instance_mats_.size() / 16;
  if (instance_vbo_ == 0) {
    initializeGl();
  }
  glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(instance_mats_.size() * sizeof(float)),
               instance_mats_.data(), GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ConeMesh::draw() const {
  if (vao_ == 0 || indices_.empty() || instance_count_ == 0) {
    return;
  }
  glBindVertexArray(vao_);
  glDrawElementsInstanced(GL_TRIANGLES,
                          static_cast<GLsizei>(indices_.size()),
                          GL_UNSIGNED_INT, nullptr,
                          static_cast<GLsizei>(instance_count_));
  glBindVertexArray(0);
}

}  // namespace fsai::io::camera::sim_stereo
