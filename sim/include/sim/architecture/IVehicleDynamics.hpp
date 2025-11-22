#pragma once

#include "Transform.h"
#include "VehicleState.hpp"
#include "WheelsInfo.h"

namespace fsai::vehicle {

/**
 * Abstraction around the vehicle dynamics implementation used by the world
 * to advance state based on commands.
 */
class IVehicleDynamics {
 public:
  virtual ~IVehicleDynamics() = default;

  // Control inputs provided by the IO/control boundary.
  virtual void set_command(float throttle, float brake, float steer) = 0;

  // Integrate dynamics by dt seconds using the last command.
  virtual void step(double dt_seconds) = 0;

  // Authoritative ground-truth state management.
  virtual void set_state(const VehicleState& state, const Transform& transform) = 0;
  virtual void reset_input() = 0;

  // Read-only accessors for world/telemetry producers.
  virtual const VehicleState& state() const = 0;
  virtual const Transform& transform() const = 0;
  virtual const WheelsInfo& wheels_info() const = 0;
};

}  // namespace fsai::vehicle

