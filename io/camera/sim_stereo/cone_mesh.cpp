#include "cone_mesh.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {

namespace {
constexpr int kSegments = 32;
constexpr float kBaseHalfExtent = 0.5f;
constexpr float kBaseHeight = 0.15f;
constexpr float kBodyBottomRadius = 0.36f;
constexpr float kBodyTopRadius = 0.1f;
constexpr float kTipHeight = 1.05f;

ConeVertex makeVertex(float x, float y, float z, float region) {
  return ConeVertex{{x, y, z}, region};
}

}  // namespace

ConeMesh::ConeMesh() {
  const std::array<std::pair<float, float>, 4> base_corners = {
      std::pair{-kBaseHalfExtent, -kBaseHalfExtent},
      std::pair{kBaseHalfExtent, -kBaseHalfExtent},
      std::pair{kBaseHalfExtent, kBaseHalfExtent},
      std::pair{-kBaseHalfExtent, kBaseHalfExtent}};

  vertices_.reserve(static_cast<std::size_t>(kSegments) * 2 + 13);

  for (const auto& [x, z] : base_corners) {
    vertices_.push_back(makeVertex(x, 0.0f, z, 0.0f));
  }
  for (const auto& [x, z] : base_corners) {
    vertices_.push_back(makeVertex(x, kBaseHeight, z, 0.0f));
  }

  const uint32_t base_top_center = static_cast<uint32_t>(vertices_.size());
  vertices_.push_back(makeVertex(0.0f, kBaseHeight, 0.0f, 0.0f));

  for (int i = 0; i < 4; ++i) {
    const uint32_t bottom0 = static_cast<uint32_t>(i);
    const uint32_t bottom1 = static_cast<uint32_t>((i + 1) % 4);
    const uint32_t top0 = static_cast<uint32_t>(4 + i);
    const uint32_t top1 = static_cast<uint32_t>(4 + ((i + 1) % 4));
    indices_.push_back(bottom0);
    indices_.push_back(bottom1);
    indices_.push_back(top1);
    indices_.push_back(bottom0);
    indices_.push_back(top1);
    indices_.push_back(top0);
    indices_.push_back(base_top_center);
    indices_.push_back(top0);
    indices_.push_back(top1);
  }

  const uint32_t body_bottom_start = static_cast<uint32_t>(vertices_.size());
  for (int i = 0; i < kSegments; ++i) {
    const float angle = static_cast<float>(i) / static_cast<float>(kSegments) *
                       static_cast<float>(2.0 * M_PI);
    const float x = kBodyBottomRadius * std::cos(angle);
    const float z = kBodyBottomRadius * std::sin(angle);
    vertices_.push_back(makeVertex(x, kBaseHeight, z, 1.0f));
  }

  const uint32_t body_top_start = static_cast<uint32_t>(vertices_.size());
  for (int i = 0; i < kSegments; ++i) {
    const float angle = static_cast<float>(i) / static_cast<float>(kSegments) *
                       static_cast<float>(2.0 * M_PI);
    const float x = kBodyTopRadius * std::cos(angle);
    const float z = kBodyTopRadius * std::sin(angle);
    vertices_.push_back(makeVertex(x, 1.0f, z, 1.0f));
  }

  const uint32_t tip_index = static_cast<uint32_t>(vertices_.size());
  vertices_.push_back(makeVertex(0.0f, kTipHeight, 0.0f, 1.0f));

  for (int i = 0; i < kSegments; ++i) {
    const uint32_t next = static_cast<uint32_t>((i + 1) % kSegments);
    indices_.push_back(body_bottom_start + static_cast<uint32_t>(i));
    indices_.push_back(body_bottom_start + next);
    indices_.push_back(body_top_start + static_cast<uint32_t>(i));
    indices_.push_back(body_top_start + static_cast<uint32_t>(i));
    indices_.push_back(body_bottom_start + next);
    indices_.push_back(body_top_start + next);
  }

  for (int i = 0; i < kSegments; ++i) {
    const uint32_t next = static_cast<uint32_t>((i + 1) % kSegments);
    indices_.push_back(tip_index);
    indices_.push_back(body_top_start + next);
    indices_.push_back(body_top_start + static_cast<uint32_t>(i));
  }
}

ConeMesh::ConeMesh(ConeMesh&& other) noexcept { *this = std::move(other); }

ConeMesh& ConeMesh::operator=(ConeMesh&& other) noexcept {
  if (this != &other) {
    destroyGl();
    vertices_ = std::move(other.vertices_);
    indices_ = std::move(other.indices_);
    instances_ = std::move(other.instances_);
    instance_buffer_ = std::move(other.instance_buffer_);
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
  glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(ConeVertex),
                        reinterpret_cast<void*>(offsetof(ConeVertex, region)));

  glGenBuffers(1, &instance_vbo_);
  glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
  glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

  const GLsizei instance_stride =
      static_cast<GLsizei>((16 + 3 + 3 + 1) * sizeof(float));

  for (int i = 0; i < 4; ++i) {
    glEnableVertexAttribArray(2 + i);
    glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, instance_stride,
                          reinterpret_cast<void*>(sizeof(float) * 4 * i));
    glVertexAttribDivisor(2 + i, 1);
  }

  const GLsizei color_offset = static_cast<GLsizei>(16 * sizeof(float));
  glEnableVertexAttribArray(6);
  glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, instance_stride,
                        reinterpret_cast<void*>(color_offset));
  glVertexAttribDivisor(6, 1);

  glEnableVertexAttribArray(7);
  glVertexAttribPointer(7, 3, GL_FLOAT, GL_FALSE, instance_stride,
                        reinterpret_cast<void*>(color_offset + 3 * sizeof(float)));
  glVertexAttribDivisor(7, 1);

  glEnableVertexAttribArray(8);
  glVertexAttribPointer(8, 1, GL_FLOAT, GL_FALSE, instance_stride,
                        reinterpret_cast<void*>(color_offset + 6 * sizeof(float)));
  glVertexAttribDivisor(8, 1);

  glBindVertexArray(0);
}

void ConeMesh::setInstances(const std::vector<ConeInstanceData>& instances) {
  instances_ = instances;
  instance_count_ = instances_.size();
  if (instance_vbo_ == 0) {
    initializeGl();
  }
  instance_buffer_.clear();
  instance_buffer_.reserve(instance_count_ * (16 + 3 + 3 + 1));
  for (const auto& instance : instances_) {
    instance_buffer_.insert(instance_buffer_.end(), instance.model.begin(),
                            instance.model.end());
    instance_buffer_.insert(instance_buffer_.end(), instance.body_color.begin(),
                            instance.body_color.end());
    instance_buffer_.insert(instance_buffer_.end(), instance.stripe_color.begin(),
                            instance.stripe_color.end());
    instance_buffer_.push_back(instance.stripe_count);
  }
  glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(instance_buffer_.size() * sizeof(float)),
               instance_buffer_.data(), GL_DYNAMIC_DRAW);
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

