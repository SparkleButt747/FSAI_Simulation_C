#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "io/camera/sim_stereo/cone_mesh.hpp"
#include "io/camera/sim_stereo/ground_plane.hpp"
#include "io/camera/sim_stereo/shader_program.hpp"

namespace fsai::io::camera::sim_stereo {

struct EyeBuffer {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> rgba;
};

class FboStereo {
 public:
  FboStereo();

  void resize(int width, int height);
  void renderScene(const ConeMesh& cones, const GroundPlane& ground,
                   const std::array<float, 16>& view_left,
                   const std::array<float, 16>& proj_left,
                   const std::array<float, 16>& view_right,
                   const std::array<float, 16>& proj_right);
  void applyNoise(float stddev, std::mt19937& rng);

  const EyeBuffer& left() const { return left_; }
  const EyeBuffer& right() const { return right_; }

 private:
  void clear(EyeBuffer& eye, uint8_t r, uint8_t g, uint8_t b);
  static std::array<float, 4> mul(const std::array<float, 16>& m,
                                  const std::array<float, 4>& v);

  EyeBuffer left_;
  EyeBuffer right_;
};

}  // namespace fsai::io::camera::sim_stereo
