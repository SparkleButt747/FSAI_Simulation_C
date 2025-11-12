#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>

/**
 * @brief Full vehicle state used by the dynamic bicycle model.
 *
 * position: [m] world frame (x forward, y left, z up)
 * velocity: [m/s] body frame (x forward, y left, z up)
 * rotation: [rad/s] body angular rates (roll, pitch, yaw)
 * acceleration: [m/s^2] body frame (used for logging; xDot writes it)
 * yaw: [rad] heading angle (world frame)
 * timestampNs: optional time tag
 */
class VehicleState {
public:
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity{0.0, 0.0, 0.0};
  Eigen::Vector3d rotation{0.0, 0.0, 0.0};
  Eigen::Vector3d acceleration{0.0, 0.0, 0.0};
  double yaw{0.0};
  std::uint64_t timestampNs{0};

  VehicleState() = default;

  VehicleState(const Eigen::Vector3d& pos,
               double yawAngle,
               const Eigen::Vector3d& vel,
               const Eigen::Vector3d& rot,
               const Eigen::Vector3d& acc,
               std::uint64_t t_ns = 0)
    : position(pos),
      velocity(vel),
      rotation(rot),
      acceleration(acc),
      yaw(yawAngle),
      timestampNs(t_ns)
  {}

  VehicleState operator*(double dt) const {
    VehicleState scaled;
    scaled.position     = position * dt;
    scaled.velocity     = velocity * dt;
    scaled.rotation     = rotation * dt;
    scaled.acceleration = acceleration * dt;
    scaled.yaw          = yaw * dt;
    scaled.timestampNs  = timestampNs;
    return scaled;
  }

  VehicleState operator+(const VehicleState& other) const {
    VehicleState result;
    result.position     = position + other.position;
    result.velocity     = velocity + other.velocity;
    result.rotation     = rotation + other.rotation;
    result.acceleration = acceleration + other.acceleration;
    result.yaw          = yaw + other.yaw;
    result.timestampNs  = std::max(timestampNs, other.timestampNs);
    return result;
  }

  std::string toString() const {
    char buffer[256];
    std::snprintf(
      buffer, sizeof(buffer),
      "pos:(%f,%f,%f) yaw:%f vel:(%f,%f,%f) rot:(%f,%f,%f) acc:(%f,%f,%f) t_ns:%llu",
      position.x(), position.y(), position.z(), yaw,
      velocity.x(), velocity.y(), velocity.z(),
      rotation.x(), rotation.y(), rotation.z(),
      acceleration.x(), acceleration.y(), acceleration.z(),
      static_cast<unsigned long long>(timestampNs)
    );
    return std::string(buffer);
  }
};

