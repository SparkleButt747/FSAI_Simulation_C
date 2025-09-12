#pragma once
#include <Eigen/Dense>
#include <string>

/**
 * @brief State of the vehicle expressed using Eigen vectors.
 *        Positions, velocities, rotations and accelerations are stored
 *        as 3D vectors while the heading (yaw) is a scalar.
 */
class VehicleState {
public:
    Eigen::Vector3d position{0.0,0.0,0.0};
    Eigen::Vector3d velocity{0.0,0.0,0.0};
    Eigen::Vector3d rotation{0.0,0.0,0.0};
    Eigen::Vector3d acceleration{0.0,0.0,0.0};
    double yaw{0.0};

    VehicleState() = default;
    VehicleState(const Eigen::Vector3d& pos,
                 double yawAngle,
                 const Eigen::Vector3d& vel,
                 const Eigen::Vector3d& rot,
                 const Eigen::Vector3d& acc)
        : position(pos), velocity(vel), rotation(rot),
          acceleration(acc), yaw(yawAngle) {}

    /// Scalar multiplication (state * dt).
    VehicleState operator*(double dt) const {
        VehicleState scaled;
        scaled.position = position * dt;
        scaled.velocity = velocity * dt;
        scaled.rotation = rotation * dt;
        scaled.acceleration = acceleration * dt;
        scaled.yaw = yaw * dt;
        return scaled;
    }

    /// Component wise addition of two states.
    VehicleState operator+(const VehicleState& other) const {
        VehicleState result;
        result.position = position + other.position;
        result.velocity = velocity + other.velocity;
        result.rotation = rotation + other.rotation;
        result.acceleration = acceleration + other.acceleration;
        result.yaw = yaw + other.yaw;
        return result;
    }

    /// Convenience to stringify the state for debugging.
    std::string toString() const {
        char buffer[256];
        std::snprintf(buffer, sizeof(buffer),
                      "pos:(%f,%f,%f) yaw:%f vel:(%f,%f,%f) rot:(%f,%f,%f) acc:(%f,%f,%f)",
                      position.x(), position.y(), position.z(), yaw,
                      velocity.x(), velocity.y(), velocity.z(),
                      rotation.x(), rotation.y(), rotation.z(),
                      acceleration.x(), acceleration.y(), acceleration.z());
        return std::string(buffer);
    }
};
