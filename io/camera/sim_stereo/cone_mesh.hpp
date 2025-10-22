#pragma once

#include <vector>

#include "io/camera/sim_stereo/shader_program.hpp"

namespace fsai::io::camera::sim_stereo {

struct ConeVertex {
  float position[3];
  float color[3];
};

// CPU representation of a low resolution cone mesh.  The geometry is static and
// only the instance transforms change when the vehicle updates the cone list.
class ConeMesh {
 public:
  ConeMesh();

  const std::vector<ConeVertex>& vertices() const { return vertices_; }
  const std::vector<uint32_t>& indices() const { return indices_; }
  const std::vector<float>& instanceMatrices() const { return instance_mats_; }

  void setInstances(const std::vector<float>& matrices4x4);
  size_t instanceCount() const { return instance_mats_.size() / 16; }

 private:
  std::vector<ConeVertex> vertices_;
  std::vector<uint32_t> indices_;
  std::vector<float> instance_mats_;
};

}  // namespace fsai::io::camera::sim_stereo
