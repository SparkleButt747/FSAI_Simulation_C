#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "types.h"
#include "cone_mesh.hpp"
#include "fbo_stereo.hpp"
#include "gl_context_sdl.hpp"
#include "ground_plane.hpp"
#include "readback_pbo.hpp"
#include "shader_program.hpp"

namespace fsai::io::camera::sim_stereo {

struct SimStereoConfig {
  int width = 640;
  int height = 480;
  float near_plane = 0.1f;
  float far_plane = 200.0f;
  float noise_stddev = 0.0f;
  FsaiCameraIntrinsics intrinsics{};
  FsaiCameraExtrinsics left_extrinsics{};
  FsaiCameraExtrinsics right_extrinsics{};
};

struct SimConeInstance {
  std::array<float, 3> position{0.0f, 0.0f, 0.0f};
  float base_width = 0.0f;
  float height = 0.0f;
  std::array<float, 3> body_color{1.0f, 0.5f, 0.0f};
  std::array<float, 3> stripe_color{1.0f, 1.0f, 1.0f};
  int stripe_count = 0;
};

class SimStereoSource {
 public:
  explicit SimStereoSource(const SimStereoConfig& config);

  void setCones(const std::vector<SimConeInstance>& cones_world);
  void setBodyPose(float x, float y, float z, float yaw);
  const FsaiStereoFrame& capture(uint64_t timestamp_ns);

  const FsaiStereoFrame& frame() const { return stereo_frame_; }

 private:
  void ensureGl();
  void updateProjection();
  void updateViews();
  void updateViewForEye(const FsaiCameraExtrinsics& extr,
                        std::array<float, 16>& out_view);
  static std::array<float, 16> projectionFromIntrinsics(
      int width, int height, float near_plane, float far_plane,
      const FsaiCameraIntrinsics& K);
  void applyNoise(std::vector<uint8_t>& rgba);

  SimStereoConfig config_;
  GlContextSdl gl_context_;
  ShaderProgram cone_shader_;
  ShaderProgram ground_shader_;
  ConeMesh cone_mesh_;
  GroundPlane ground_;
  FboStereo fbo_;
  ReadbackPbo pbo_left_;
  ReadbackPbo pbo_right_;
  std::array<float, 16> view_left_{};
  std::array<float, 16> view_right_{};
  std::array<float, 16> proj_left_{};
  std::array<float, 16> proj_right_{};
  std::vector<uint8_t> left_rgba_;
  std::vector<uint8_t> right_rgba_;
  std::vector<uint8_t> left_pixels_;
  std::vector<uint8_t> right_pixels_;
  FsaiStereoFrame stereo_frame_{};
  std::mt19937 rng_;
  std::array<float, 3> body_position_{{0.0f, 1.0f, 0.0f}};
  float body_yaw_ = 0.0f;
};

}  // namespace fsai::io::camera::sim_stereo
