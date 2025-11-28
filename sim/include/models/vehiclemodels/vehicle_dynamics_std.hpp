#pragma once

#include <vector>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * vehicle_dynamics_std - single-track drift model vehicle dynamics.
 *
 * State x:
 *   x1 = x-position in a global coordinate system
 *   x2 = y-position in a global coordinate system
 *   x3 = steering angle of front wheels
 *   x4 = velocity at vehicle center
 *   x5 = yaw angle
 *   x6 = yaw rate
 *   x7 = slip angle at vehicle center
 *   x8 = front wheel angular speed
 *   x9 = rear wheel angular speed
 *
 * Input u_init:
 *   u1 = steering angle velocity of front wheels
 *   u2 = longitudinal acceleration
 *
 * Returns:
 *   f = right-hand side of differential equations
 */
std::vector<double> vehicle_dynamics_std(const std::vector<double>& x,
                                         const std::vector<double>& u_init,
                                         const VehicleParameters& p,
                                         double dt);

std::vector<double> vehicle_dynamics_std(const std::vector<double>& x,
                                         const std::vector<double>& u_init,
                                         const VehicleParameters& p);

} // namespace velox::models
