#include "io/camera/sim_stereo/ground_plane.hpp"

namespace fsai::io::camera::sim_stereo {

namespace {
GroundVertex makeGroundVertex(float x, float y, float z, float r, float g, float b) {
  return GroundVertex{{x, y, z}, {r, g, b}};
}
}  // namespace

GroundPlane::GroundPlane() {
  vertices_.reserve(4);
  indices_.reserve(6);

  vertices_.push_back(makeGroundVertex(-5.0f, -5.0f, 0.0f, 0.2f, 0.2f, 0.2f));
  vertices_.push_back(makeGroundVertex(5.0f, -5.0f, 0.0f, 0.2f, 0.2f, 0.2f));
  vertices_.push_back(makeGroundVertex(5.0f, 5.0f, 0.0f, 0.3f, 0.3f, 0.3f));
  vertices_.push_back(makeGroundVertex(-5.0f, 5.0f, 0.0f, 0.3f, 0.3f, 0.3f));

  indices_.insert(indices_.end(), {0, 1, 2, 0, 2, 3});
}

}  // namespace fsai::io::camera::sim_stereo
