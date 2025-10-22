#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fsai::io::camera::sim_stereo {

struct ConeVertex {
  std::array<float, 3> position;
  std::array<float, 3> color;
};

class ConeMesh {
 public:
  ConeMesh();
  ConeMesh(const ConeMesh&) = delete;
  ConeMesh& operator=(const ConeMesh&) = delete;
  ConeMesh(ConeMesh&& other) noexcept;
  ConeMesh& operator=(ConeMesh&& other) noexcept;
  ~ConeMesh();

  void initializeGl();
  void draw() const;

  void setInstances(const std::vector<float>& matrices4x4);
  std::size_t instanceCount() const { return instance_count_; }

 private:
  void uploadGeometry();
  void destroyGl();

  std::vector<ConeVertex> vertices_;
  std::vector<uint32_t> indices_;
  std::vector<float> instance_mats_;
  std::size_t instance_count_ = 0;

  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int ebo_ = 0;
  unsigned int instance_vbo_ = 0;
};

}  // namespace fsai::io::camera::sim_stereo
