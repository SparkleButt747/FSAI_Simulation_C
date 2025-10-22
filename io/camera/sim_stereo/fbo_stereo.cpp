#include "io/camera/sim_stereo/fbo_stereo.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::io::camera::sim_stereo {

namespace {
void drawDisk(EyeBuffer& eye, int cx, int cy, int radius, uint8_t r, uint8_t g,
              uint8_t b) {
  if (eye.width <= 0 || eye.height <= 0) {
    return;
  }
  int r2 = radius * radius;
  for (int y = cy - radius; y <= cy + radius; ++y) {
    if (y < 0 || y >= eye.height) continue;
    for (int x = cx - radius; x <= cx + radius; ++x) {
      if (x < 0 || x >= eye.width) continue;
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy > r2) continue;
      size_t idx = static_cast<size_t>(y) * static_cast<size_t>(eye.width) * 4 +
                   static_cast<size_t>(x) * 4;
      eye.rgba[idx + 0] = r;
      eye.rgba[idx + 1] = g;
      eye.rgba[idx + 2] = b;
      eye.rgba[idx + 3] = 255;
    }
  }
}
}  // namespace

FboStereo::FboStereo() { resize(640, 480); }

void FboStereo::resize(int width, int height) {
  auto ensure = [&](EyeBuffer& eye) {
    if (eye.width != width || eye.height != height) {
      eye.width = width;
      eye.height = height;
      eye.rgba.assign(static_cast<size_t>(width) * static_cast<size_t>(height) *
                          4,
                      0);
    }
  };
  ensure(left_);
  ensure(right_);
}

void FboStereo::clear(EyeBuffer& eye, uint8_t r, uint8_t g, uint8_t b) {
  if (eye.width <= 0 || eye.height <= 0) {
    return;
  }
  for (int y = 0; y < eye.height; ++y) {
    float t = static_cast<float>(y) / static_cast<float>(std::max(1, eye.height - 1));
    uint8_t rr = static_cast<uint8_t>(std::clamp(r * (1.0f - 0.3f * t), 0.0f, 255.0f));
    uint8_t gg = static_cast<uint8_t>(std::clamp(g * (1.0f - 0.3f * t), 0.0f, 255.0f));
    uint8_t bb = static_cast<uint8_t>(std::clamp(b * (1.0f - 0.3f * t), 0.0f, 255.0f));
    for (int x = 0; x < eye.width; ++x) {
      size_t idx = static_cast<size_t>(y) * static_cast<size_t>(eye.width) * 4 +
                   static_cast<size_t>(x) * 4;
      eye.rgba[idx + 0] = rr;
      eye.rgba[idx + 1] = gg;
      eye.rgba[idx + 2] = bb;
      eye.rgba[idx + 3] = 255;
    }
  }
}

std::array<float, 4> FboStereo::mul(const std::array<float, 16>& m,
                                    const std::array<float, 4>& v) {
  std::array<float, 4> out{};
  for (int row = 0; row < 4; ++row) {
    out[row] = m[row * 4 + 0] * v[0] + m[row * 4 + 1] * v[1] +
               m[row * 4 + 2] * v[2] + m[row * 4 + 3] * v[3];
  }
  return out;
}

void FboStereo::renderScene(const ConeMesh& cones, const GroundPlane& /*ground*/,
                            const std::array<float, 16>& view_left,
                            const std::array<float, 16>& proj_left,
                            const std::array<float, 16>& view_right,
                            const std::array<float, 16>& proj_right) {
  clear(left_, 180, 200, 220);
  clear(right_, 180, 200, 220);

  const auto& instances = cones.instanceMatrices();
  const size_t num_instances = cones.instanceCount();

  auto drawWith = [&](EyeBuffer& eye, const std::array<float, 16>& view,
                      const std::array<float, 16>& proj) {
    for (size_t i = 0; i < num_instances; ++i) {
      std::array<float, 16> model{};
      for (int j = 0; j < 16; ++j) {
        model[j] = instances[i * 16 + j];
      }
      auto p = mul(model, {0.0f, 0.0f, 0.0f, 1.0f});
      auto v = mul(view, {p[0], p[1], p[2], 1.0f});
      auto clip = mul(proj, {v[0], v[1], v[2], 1.0f});
      if (clip[3] <= 0.0f) {
        continue;
      }
      float invw = 1.0f / clip[3];
      float ndc_x = clip[0] * invw;
      float ndc_y = clip[1] * invw;
      if (ndc_x < -1.2f || ndc_x > 1.2f || ndc_y < -1.2f || ndc_y > 1.2f) {
        continue;
      }
      int px = static_cast<int>((ndc_x * 0.5f + 0.5f) * static_cast<float>(eye.width));
      int py = static_cast<int>((-ndc_y * 0.5f + 0.5f) * static_cast<float>(eye.height));
      drawDisk(eye, px, py, 6, 255, 180, 60);
    }
  };

  drawWith(left_, view_left, proj_left);
  drawWith(right_, view_right, proj_right);
}

void FboStereo::applyNoise(float stddev, std::mt19937& rng) {
  if (stddev <= 0.0f) {
    return;
  }
  std::normal_distribution<float> dist(0.0f, stddev);
  auto apply = [&](EyeBuffer& eye) {
    for (size_t i = 0; i + 3 < eye.rgba.size(); i += 4) {
      for (int c = 0; c < 3; ++c) {
        float val = static_cast<float>(eye.rgba[i + c]) + dist(rng);
        val = std::clamp(val, 0.0f, 255.0f);
        eye.rgba[i + c] = static_cast<uint8_t>(val);
      }
    }
  };
  apply(left_);
  apply(right_);
}

}  // namespace fsai::io::camera::sim_stereo
