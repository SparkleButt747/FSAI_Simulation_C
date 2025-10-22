#include "io/camera/sim_stereo/cone_mesh.hpp"

#include <cmath>

namespace fsai::io::camera::sim_stereo {

namespace {
constexpr int kSegments = 16;

ConeVertex makeVertex(float x, float y, float z, float r, float g, float b) {
  return ConeVertex{{x, y, z}, {r, g, b}};
}
}  // namespace

ConeMesh::ConeMesh() {
  vertices_.reserve(kSegments + 1);

  // Apex at the top.
  vertices_.push_back(makeVertex(0.0f, 0.0f, 0.5f, 1.0f, 0.6f, 0.1f));

  for (int i = 0; i < kSegments; ++i) {
    float angle = static_cast<float>(i) / static_cast<float>(kSegments) *
                  static_cast<float>(2.0 * M_PI);
    float x = 0.25f * std::cos(angle);
    float y = 0.25f * std::sin(angle);
    vertices_.push_back(makeVertex(x, y, 0.0f, 1.0f, 0.8f, 0.3f));
  }

  // Build index buffer for the cone sides.
  for (int i = 0; i < kSegments; ++i) {
    uint32_t next = (i + 1) % kSegments;
    indices_.push_back(0);
    indices_.push_back(1 + next);
    indices_.push_back(1 + i);
  }

  // Base (optional) - provide a simple fan for completeness.
  uint32_t base_center = static_cast<uint32_t>(vertices_.size());
  vertices_.push_back(makeVertex(0.0f, 0.0f, 0.0f, 0.4f, 0.4f, 0.4f));
  for (int i = 0; i < kSegments; ++i) {
    uint32_t next = (i + 1) % kSegments;
    indices_.push_back(base_center);
    indices_.push_back(1 + i);
    indices_.push_back(1 + next);
  }
}

void ConeMesh::setInstances(const std::vector<float>& matrices4x4) {
  instance_mats_ = matrices4x4;
}

}  // namespace fsai::io::camera::sim_stereo
