#pragma once

#include <vector>

namespace fsai::io::camera::sim_stereo {

struct GroundVertex {
  float position[3];
  float color[3];
};

class GroundPlane {
 public:
  GroundPlane();

  const std::vector<GroundVertex>& vertices() const { return vertices_; }
  const std::vector<uint32_t>& indices() const { return indices_; }

 private:
  std::vector<GroundVertex> vertices_;
  std::vector<uint32_t> indices_;
};

}  // namespace fsai::io::camera::sim_stereo
