#pragma once

#include <array>
#include <string_view>

#include <Eigen/Core>

#include "Transform.h"
#include "Vector.h"

namespace fsai::sim {

struct FrameConvention {
  std::string_view name;
  std::array<double, 3> x_axis;
  std::array<double, 3> y_axis;
  std::array<double, 3> z_axis;
  std::string_view notes;
};

inline constexpr FrameConvention kPhysicsWorldFrame{
  "physics_world",
  {1.0, 0.0, 0.0},  // x: forward/east
  {0.0, 1.0, 0.0},  // y: left/north
  {0.0, 0.0, 1.0},  // z: up
  "Right-handed frame used by DynamicBicycle: x forward, y left, z up"
};

inline constexpr FrameConvention kTrackFrame{
  "track_render",
  {1.0, 0.0, 0.0},  // x: east/right
  {0.0, 0.0, 1.0},  // y: up
  {0.0, 1.0, 0.0},  // z: north/forward
  "Right-handed frame used by track generation & controller: XZ lie in ground plane"
};

inline constexpr FrameConvention kSdlScreenFrame{
  "sdl_screen",
  {1.0, 0.0, 0.0},
  {0.0, -1.0, 0.0},
  {0.0, 0.0, 1.0},
  "2D screen coordinates: x right, y down, z out of screen"
};

inline Eigen::Vector3d trackPointToWorldPosition(const Vector3& track) {
  return Eigen::Vector3d(static_cast<double>(track.x),
                         static_cast<double>(track.z),
                         static_cast<double>(track.y));
}

inline Eigen::Vector3d trackTransformToWorldPosition(const Transform& t) {
  return trackPointToWorldPosition(t.position);
}

inline Vector3 worldPositionToTrackPoint(const Eigen::Vector3d& world) {
  Vector3 out{};
  out.x = static_cast<float>(world.x());
  out.y = static_cast<float>(world.z());
  out.z = static_cast<float>(world.y());
  return out;
}

inline Transform worldStateToTransform(const Eigen::Vector3d& world,
                                       float yaw_rad,
                                       float height_override = 0.0f) {
  Transform out{};
  out.position = worldPositionToTrackPoint(world);
  if (height_override != 0.0f) {
    out.position.y = height_override;
  }
  out.yaw = yaw_rad;
  return out;
}

inline Eigen::Vector2d trackXZToWorldXY(const Vector2& track) {
  return Eigen::Vector2d(static_cast<double>(track.x), static_cast<double>(track.y));
}

inline Vector2 worldXYToTrackXZ(const Eigen::Vector2d& world) {
  Vector2 out{static_cast<float>(world.x()), static_cast<float>(world.y())};
  return out;
}

inline Vector2 trackDifference(const Vector3& a, const Vector3& b) {
  return Vector2{a.x - b.x, a.z - b.z};
}

}  // namespace fsai::sim

