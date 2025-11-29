#pragma once

#include <array>

#include "models/acceleration_constraints.hpp"
#include "models/steering_constraints.hpp"
#include "vehicle_parameters.hpp"  // assumes VehicleParameters in namespace velox::models

namespace velox::models {
namespace utils {

/**
 * vehicle_dynamics_ks_cog - kinematic single-track vehicle dynamics
 * reference point: center of mass
 *
 * State x:
 *   x[0] = x-position in a global coordinate system
 *   x[1] = y-position in a global coordinate system
 *   x[2] = steering angle of front wheels
 *   x[3] = velocity at center of mass
 *   x[4] = yaw angle
 *
 * Input u_init:
 *   u_init[0] = steering velocity
 *   u_init[1] = acceleration
 */
std::array<double, 5> vehicle_dynamics_ks_cog(const std::array<double, 5>& x,
                                              const std::array<double, 2>& u_init,
                                              const ::velox::models::VehicleParameters& p);

} // namespace utils
} // namespace velox::models
