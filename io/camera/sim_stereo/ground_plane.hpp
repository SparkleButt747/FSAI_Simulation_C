#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace fsai::io::camera::sim_stereo {

struct GroundVertex {
  std::array<float, 3> position;
  std::array<float, 3> color;
};

class GroundPlane {
 public:
  GroundPlane();
  GroundPlane(const GroundPlane&) = delete;
  GroundPlane& operator=(const GroundPlane&) = delete;
  GroundPlane(GroundPlane&& other) noexcept;
  GroundPlane& operator=(GroundPlane&& other) noexcept;
  ~GroundPlane();

  void initializeGl();
  void draw() const;

 private:
  void uploadGeometry();
  void destroyGl();

  std::vector<GroundVertex> vertices_;
  std::vector<uint32_t> indices_;

  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int ebo_ = 0;
};

}  // namespace fsai::io::camera::sim_stereo
