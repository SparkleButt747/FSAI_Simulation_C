#include "sim_stereo_source.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>

#include <Eigen/Dense>
#include <SDL.h>
#include <glad/glad.h>

namespace fsai::io::camera::sim_stereo {
namespace {
std::array<float, 16> eigenToArray(const Eigen::Matrix4f& m) {
  std::array<float, 16> out{};
  std::copy(m.data(), m.data() + 16, out.begin());
  return out;
}
}

SimStereoSource::SimStereoSource(const SimStereoConfig& config)
    : config_(config) {
  rng_.seed(std::random_device{}());
  ensureGl();

  cone_shader_ = buildDefaultConeShader();
  ground_shader_ = buildDefaultGroundShader();

  cone_mesh_.initializeGl();
  ground_.initializeGl();

  updateProjection();
  updateViews();

  fbo_.resize(config_.width, config_.height);

  stereo_frame_.left.fmt = FSAI_PIXEL_RGB888;
  stereo_frame_.right.fmt = FSAI_PIXEL_RGB888;
  stereo_frame_.left.K = config_.intrinsics;
  stereo_frame_.right.K = config_.intrinsics;
  stereo_frame_.left.T_bw = config_.left_extrinsics;
  stereo_frame_.right.T_bw = config_.right_extrinsics;
}

void SimStereoSource::ensureGl() {
  gl_context_.makeCurrent();
  static bool glad_loaded = false;
  if (!glad_loaded) {
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(SDL_GL_GetProcAddress))) {
      throw std::runtime_error("Failed to load OpenGL functions");
    }
    glad_loaded = true;
  }
}

void SimStereoSource::setCones(
    const std::vector<std::array<float, 3>>& cones_world) {
  gl_context_.makeCurrent();
  std::vector<float> matrices;
  matrices.reserve(cones_world.size() * 16);
  for (const auto& p : cones_world) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model(0, 3) = p[0];
    model(1, 3) = p[1];
    model(2, 3) = p[2];
    auto arr = eigenToArray(model);
    matrices.insert(matrices.end(), arr.begin(), arr.end());
  }
  cone_mesh_.setInstances(matrices);
}

void SimStereoSource::setBodyPose(float x, float y, float z, float yaw) {
  body_position_[0] = x;
  body_position_[1] = y;
  body_position_[2] = z;
  body_yaw_ = yaw;
  updateViews();
}

void SimStereoSource::updateProjection() {
  proj_left_ = projectionFromIntrinsics(config_.width, config_.height,
                                        config_.near_plane, config_.far_plane,
                                        config_.intrinsics);
  proj_right_ = proj_left_;
}

void SimStereoSource::updateViewForEye(const FsaiCameraExtrinsics& extr,
                                       std::array<float, 16>& out_view) {
  Eigen::Matrix3f R_bc;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_bc(r, c) = extr.R[r * 3 + c];
    }
  }

  // The legacy 2D simulation defines positive yaw as a clockwise rotation when
  // looking down onto the track (screen-space +y points downward). The OpenGL
  // renderer uses a conventional right-handed coordinate system where positive
  // rotations about the world-up axis (Y) are counter-clockwise. Negate the yaw
  // when forming the body orientation so both renderers turn in the same
  // direction.
  Eigen::Matrix3f R_wb =
      Eigen::AngleAxisf(-body_yaw_, Eigen::Vector3f::UnitY()).toRotationMatrix();
  Eigen::Matrix3f R_wc = R_wb * R_bc;

  Eigen::Vector3f t_bc(extr.t[0], extr.t[1], extr.t[2]);
  Eigen::Vector3f t_wb(body_position_[0], body_position_[1], body_position_[2]);
  Eigen::Vector3f t_wc = R_wb * t_bc + t_wb;

  Eigen::Matrix4f world_from_camera = Eigen::Matrix4f::Identity();
  world_from_camera.block<3, 3>(0, 0) = R_wc;
  world_from_camera.block<3, 1>(0, 3) = t_wc;

  Eigen::Matrix4f view = world_from_camera.inverse();
  out_view = eigenToArray(view);
}

void SimStereoSource::updateViews() {
  updateViewForEye(config_.left_extrinsics, view_left_);
  updateViewForEye(config_.right_extrinsics, view_right_);
}

std::array<float, 16> SimStereoSource::projectionFromIntrinsics(
    int width, int height, float near_plane, float far_plane,
    const FsaiCameraIntrinsics& K) {
  float aspect = static_cast<float>(width) / static_cast<float>(height);
  float fy = K.fy != 0.0f ? K.fy : static_cast<float>(height);
  float fovy = 2.0f * std::atan(static_cast<float>(height) / (2.0f * fy));
  float f = 1.0f / std::tan(fovy / 2.0f);

  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / aspect;
  proj(1, 1) = f;
  proj(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
  proj(2, 3) = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
  proj(3, 2) = -1.0f;

  proj(0, 2) = 2.0f * (K.cx / static_cast<float>(width)) - 1.0f;
  proj(1, 2) = 1.0f - 2.0f * (K.cy / static_cast<float>(height));

  return eigenToArray(proj);
}

void SimStereoSource::applyNoise(std::vector<uint8_t>& rgba) {
  if (config_.noise_stddev <= 0.0f) {
    return;
  }
  std::normal_distribution<float> dist(0.0f, config_.noise_stddev);
  for (std::size_t i = 0; i + 3 < rgba.size(); i += 4) {
    for (int c = 0; c < 3; ++c) {
      float value = static_cast<float>(rgba[i + c]) + dist(rng_);
      value = std::clamp(value, 0.0f, 255.0f);
      rgba[i + c] = static_cast<uint8_t>(value);
    }
  }
}

const FsaiStereoFrame& SimStereoSource::capture(uint64_t timestamp_ns) {
  ensureGl();
  fbo_.resize(config_.width, config_.height);
  fbo_.renderScene(cone_mesh_, ground_, cone_shader_, ground_shader_, view_left_,
                   proj_left_, view_right_, proj_right_);

  pbo_left_.read(fbo_.leftFbo(), config_.width, config_.height, left_rgba_);
  pbo_right_.read(fbo_.rightFbo(), config_.width, config_.height, right_rgba_);

  applyNoise(left_rgba_);
  applyNoise(right_rgba_);

  const int width = config_.width;
  const int height = config_.height;
  const std::size_t pixel_count = static_cast<std::size_t>(width) *
                                  static_cast<std::size_t>(height);
  left_pixels_.resize(pixel_count * 3);
  right_pixels_.resize(pixel_count * 3);
  for (int y = 0; y < height; ++y) {
    const int src_y = height - 1 - y;
    for (int x = 0; x < width; ++x) {
      const std::size_t src_idx =
          (static_cast<std::size_t>(src_y) * width + static_cast<std::size_t>(x)) *
          4u;
      const std::size_t dst_idx =
          (static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)) * 3u;

      left_pixels_[dst_idx + 0] = left_rgba_[src_idx + 0];
      left_pixels_[dst_idx + 1] = left_rgba_[src_idx + 1];
      left_pixels_[dst_idx + 2] = left_rgba_[src_idx + 2];

      right_pixels_[dst_idx + 0] = right_rgba_[src_idx + 0];
      right_pixels_[dst_idx + 1] = right_rgba_[src_idx + 1];
      right_pixels_[dst_idx + 2] = right_rgba_[src_idx + 2];
    }
  }

  stereo_frame_.t_sync_ns = timestamp_ns;

  stereo_frame_.left.t_ns = timestamp_ns;
  stereo_frame_.left.w = config_.width;
  stereo_frame_.left.h = config_.height;
  stereo_frame_.left.stride = config_.width * 3;
  stereo_frame_.left.data = left_pixels_.data();

  stereo_frame_.right.t_ns = timestamp_ns;
  stereo_frame_.right.w = config_.width;
  stereo_frame_.right.h = config_.height;
  stereo_frame_.right.stride = config_.width * 3;
  stereo_frame_.right.data = right_pixels_.data();

  return stereo_frame_;
}

}  // namespace fsai::io::camera::sim_stereo
