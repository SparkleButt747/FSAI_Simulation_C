#include "io/camera/sim_stereo/sim_stereo_source.hpp"

#include <array>
#include <cmath>
#include <random>

namespace fsai::io::camera::sim_stereo {

namespace {
std::array<float, 16> identityMatrix() {
  std::array<float, 16> m{};
  m[0] = m[5] = m[10] = m[15] = 1.0f;
  return m;
}

std::array<float, 16> translationMatrix(float x, float y, float z) {
  auto m = identityMatrix();
  m[3] = x;
  m[7] = y;
  m[11] = z;
  return m;
}

}  // namespace

SimStereoSource::SimStereoSource(const SimStereoConfig& config)
    : config_(config),
      cone_shader_(buildDefaultConeShader()),
      ground_shader_(buildDefaultGroundShader()) {
  rng_.seed(std::random_device{}());
  updateProjection();
  updateViews();

  stereo_frame_.left.fmt = FSAI_PIXEL_RGB888;
  stereo_frame_.right.fmt = FSAI_PIXEL_RGB888;
  stereo_frame_.left.K = config_.intrinsics;
  stereo_frame_.right.K = config_.intrinsics;
  stereo_frame_.left.T_bw = config_.left_extrinsics;
  stereo_frame_.right.T_bw = config_.right_extrinsics;
}

void SimStereoSource::updateProjection() {
  proj_left_ = projectionFromIntrinsics(config_.width, config_.height,
                                        config_.near_plane, config_.far_plane,
                                        config_.intrinsics);
  proj_right_ = proj_left_;
}

void SimStereoSource::updateViews() {
  view_left_ = cameraMatrixFromExtrinsics(config_.left_extrinsics);
  view_right_ = cameraMatrixFromExtrinsics(config_.right_extrinsics);
}

std::array<float, 16> SimStereoSource::cameraMatrixFromExtrinsics(
    const FsaiCameraExtrinsics& extr) {
  std::array<float, 9> R{};
  for (int i = 0; i < 9; ++i) {
    R[i] = extr.R[i];
  }
  // R_bc is row-major rotation body <- camera.  For a view matrix we need
  // camera <- body, which is the transpose.
  std::array<float, 9> R_cb{};
  R_cb[0] = R[0];
  R_cb[1] = R[3];
  R_cb[2] = R[6];
  R_cb[3] = R[1];
  R_cb[4] = R[4];
  R_cb[5] = R[7];
  R_cb[6] = R[2];
  R_cb[7] = R[5];
  R_cb[8] = R[8];

  std::array<float, 16> view = identityMatrix();
  view[0] = R_cb[0];
  view[1] = R_cb[1];
  view[2] = R_cb[2];
  view[4] = R_cb[3];
  view[5] = R_cb[4];
  view[6] = R_cb[5];
  view[8] = R_cb[6];
  view[9] = R_cb[7];
  view[10] = R_cb[8];

  float tx = extr.t[0];
  float ty = extr.t[1];
  float tz = extr.t[2];
  view[3] = -(R_cb[0] * tx + R_cb[1] * ty + R_cb[2] * tz);
  view[7] = -(R_cb[3] * tx + R_cb[4] * ty + R_cb[5] * tz);
  view[11] = -(R_cb[6] * tx + R_cb[7] * ty + R_cb[8] * tz);

  return view;
}

std::array<float, 16> SimStereoSource::projectionFromIntrinsics(
    int width, int height, float near_plane, float far_plane,
    const FsaiCameraIntrinsics& K) {
  std::array<float, 16> proj{};
  proj[15] = 0.0f;

  float w = static_cast<float>(width);
  float h = static_cast<float>(height);

  proj[0] = 2.0f * K.fx / w;
  proj[1] = 0.0f;
  proj[2] = 2.0f * K.cx / w - 1.0f;
  proj[3] = 0.0f;

  proj[4] = 0.0f;
  proj[5] = -2.0f * K.fy / h;
  proj[6] = 1.0f - 2.0f * K.cy / h;
  proj[7] = 0.0f;

  proj[8] = 0.0f;
  proj[9] = 0.0f;
  if (std::abs(far_plane - near_plane) > 1e-5f) {
    proj[10] = (far_plane + near_plane) / (near_plane - far_plane);
    proj[11] = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
  } else {
    proj[10] = 1.0f;
    proj[11] = 0.0f;
  }

  proj[12] = 0.0f;
  proj[13] = 0.0f;
  proj[14] = 1.0f;
  proj[15] = 0.0f;

  return proj;
}

void SimStereoSource::setCones(
    const std::vector<std::array<float, 3>>& cones_world) {
  std::vector<float> matrices;
  matrices.reserve(cones_world.size() * 16);
  for (const auto& p : cones_world) {
    auto model = translationMatrix(p[0], p[1], p[2]);
    matrices.insert(matrices.end(), model.begin(), model.end());
  }
  cone_mesh_.setInstances(matrices);
}

const FsaiStereoFrame& SimStereoSource::capture(uint64_t timestamp_ns) {
  fbo_.resize(config_.width, config_.height);
  fbo_.renderScene(cone_mesh_, ground_, view_left_, proj_left_, view_right_,
                   proj_right_);
  fbo_.applyNoise(config_.noise_stddev, rng_);

  pbo_left_.read(fbo_.left(), left_rgba_);
  pbo_right_.read(fbo_.right(), right_rgba_);

  size_t pixel_count = static_cast<size_t>(config_.width) *
                       static_cast<size_t>(config_.height);
  left_pixels_.resize(pixel_count * 3);
  right_pixels_.resize(pixel_count * 3);
  for (size_t i = 0; i < pixel_count; ++i) {
    left_pixels_[i * 3 + 0] = left_rgba_[i * 4 + 0];
    left_pixels_[i * 3 + 1] = left_rgba_[i * 4 + 1];
    left_pixels_[i * 3 + 2] = left_rgba_[i * 4 + 2];

    right_pixels_[i * 3 + 0] = right_rgba_[i * 4 + 0];
    right_pixels_[i * 3 + 1] = right_rgba_[i * 4 + 1];
    right_pixels_[i * 3 + 2] = right_rgba_[i * 4 + 2];
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
