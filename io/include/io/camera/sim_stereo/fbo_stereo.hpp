#pragma once

#include <array>

#include <io/camera/sim_stereo/cone_mesh.hpp>
#include <io/camera/sim_stereo/ground_plane.hpp>
#include <io/camera/sim_stereo/shader_program.hpp>

namespace fsai::io::camera::sim_stereo {

class FboStereo {
 public:
  FboStereo();
  FboStereo(const FboStereo&) = delete;
  FboStereo& operator=(const FboStereo&) = delete;
  FboStereo(FboStereo&& other) noexcept;
  FboStereo& operator=(FboStereo&& other) noexcept;
  ~FboStereo();

  void resize(int width, int height);
  void renderScene(const ConeMesh& cones, const GroundPlane& ground,
                   ShaderProgram& cone_shader, ShaderProgram& ground_shader,
                   const std::array<float, 16>& view_left,
                   const std::array<float, 16>& proj_left,
                   const std::array<float, 16>& view_right,
                   const std::array<float, 16>& proj_right);

  unsigned int leftFbo() const { return left_.fbo; }
  unsigned int rightFbo() const { return right_.fbo; }
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  struct EyeTarget {
    unsigned int fbo = 0;
    unsigned int color = 0;
    unsigned int depth = 0;
  };

  void ensureEye(EyeTarget& eye);
  void destroyEye(EyeTarget& eye);
  void renderEye(EyeTarget& eye, const std::array<float, 16>& view,
                 const std::array<float, 16>& proj, ShaderProgram& cone_shader,
                 ShaderProgram& ground_shader, const ConeMesh& cones,
                 const GroundPlane& ground);

  EyeTarget left_;
  EyeTarget right_;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace fsai::io::camera::sim_stereo
