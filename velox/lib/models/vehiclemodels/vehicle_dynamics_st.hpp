#pragma once

#include <vector>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * vehicle_dynamics_st - single-track vehicle dynamics
 * reference point: center of mass.
 *
 * State x:
 *   x1 = x-position in a global coordinate system
 *   x2 = y-position in a global coordinate system
 *   x3 = steering angle of front wheels
 *   x4 = velocity in x-direction
 *   x5 = yaw angle
 *   x6 = yaw rate
 *   x7 = slip angle at vehicle center
 *
 * Input u_init:
 *   u1 = steering angle velocity of front wheels
 *   u2 = longitudinal acceleration
 *
 * Returns:
 *   f = right-hand side of differential equations
 */
std::vector<double> vehicle_dynamics_st(const std::vector<double>& x,
                                        const std::vector<double>& u_init,
                                        const VehicleParameters& p);

} // namespace velox::models
