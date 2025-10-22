#include "provider_registry.hpp"

#include <memory>
#include <mutex>

namespace fsai::sim::integration {

namespace {
std::unordered_map<std::string, StereoProviderFactory>& registry() {
  static std::unordered_map<std::string, StereoProviderFactory> map;
  return map;
}

std::mutex& registryMutex() {
  static std::mutex m;
  return m;
}

io::camera::sim_stereo::SimStereoConfig defaultConfig() {
  io::camera::sim_stereo::SimStereoConfig cfg;
  cfg.width = 640;
  cfg.height = 480;
  cfg.near_plane = 0.1f;
  cfg.far_plane = 100.0f;
  cfg.intrinsics.fx = 400.0f;
  cfg.intrinsics.fy = 400.0f;
  cfg.intrinsics.cx = 320.0f;
  cfg.intrinsics.cy = 240.0f;

  // Identity rotations.
  for (int i = 0; i < 9; ++i) {
    cfg.left_extrinsics.R[i] = 0.0f;
    cfg.right_extrinsics.R[i] = 0.0f;
  }
  cfg.left_extrinsics.R[0] = cfg.left_extrinsics.R[4] =
      cfg.left_extrinsics.R[8] = 1.0f;
  cfg.right_extrinsics.R[0] = cfg.right_extrinsics.R[4] =
      cfg.right_extrinsics.R[8] = 1.0f;

  cfg.left_extrinsics.t[0] = 0.0f;
  cfg.left_extrinsics.t[1] = 0.0f;
  cfg.left_extrinsics.t[2] = 0.0f;

  cfg.right_extrinsics.t[0] = 0.2f;
  cfg.right_extrinsics.t[1] = 0.0f;
  cfg.right_extrinsics.t[2] = 0.0f;

  return cfg;
}

}  // namespace

void registerStereoProvider(const std::string& name,
                            StereoProviderFactory factory) {
  std::lock_guard<std::mutex> lock(registryMutex());
  registry()[name] = std::move(factory);
}

StereoProviderFactory lookupStereoProvider(const std::string& name) {
  std::lock_guard<std::mutex> lock(registryMutex());
  auto it = registry().find(name);
  if (it != registry().end()) {
    return it->second;
  }
  return StereoProviderFactory{};
}

void registerBuiltInStereoProviders() {
  registerStereoProvider(
      "sim_stereo", []() {
        return std::make_unique<io::camera::sim_stereo::SimStereoSource>(
            defaultConfig());
      });
}

}  // namespace fsai::sim::integration
