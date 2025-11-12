#pragma once

#include <string>
#include <cstdio>

/**
 * @brief Driver/control input to the vehicle model.
 *
 * acc   in [-1, 1]: negative = brake, positive = throttle
 * vel   [m/s]: optional desired velocity (not used by DynamicBicycle)
 * delta [rad]: steering angle at front axle
 */
class VehicleInput {
public:
  double acc{0.0};
  double vel{0.0};
  double delta{0.0};

  VehicleInput() = default;

  VehicleInput(double acceleration, double velocity, double steering)
    : acc(acceleration), vel(velocity), delta(steering) {}

  std::string toString() const {
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer),
                  "acc:%f vel:%f delta:%f", acc, vel, delta);
    return std::string(buffer);
  }
};

