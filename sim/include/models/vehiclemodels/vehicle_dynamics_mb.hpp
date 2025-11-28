#pragma once

#include <vector>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * vehicle_dynamics_mb - multi-body vehicle dynamics based on the DOT
 * (Department of Transportation) vehicle dynamics.
 * Reference point: center of mass.
 *
 * State x (29 states, Python naming x1..x29):
 *   x1  = x-position in a global coordinate system
 *   x2  = y-position in a global coordinate system
 *   x3  = steering angle of front wheels
 *   x4  = velocity in x-direction
 *   x5  = yaw angle
 *   x6  = yaw rate
 *
 *   x7  = roll angle
 *   x8  = roll rate
 *   x9  = pitch angle
 *   x10 = pitch rate
 *   x11 = velocity in y-direction
 *   x12 = z-position
 *   x13 = velocity in z-direction
 *
 *   x14 = roll angle front
 *   x15 = roll rate front
 *   x16 = velocity in y-direction front
 *   x17 = z-position front
 *   x18 = velocity in z-direction front
 *
 *   x19 = roll angle rear
 *   x20 = roll rate rear
 *   x21 = velocity in y-direction rear
 *   x22 = z-position rear
 *   x23 = velocity in z-direction rear
 *
 *   x24 = left front wheel angular speed
 *   x25 = right front wheel angular speed
 *   x26 = left rear wheel angular speed
 *   x27 = right rear wheel angular speed
 *
 *   x28 = delta_y_f
 *   x29 = delta_y_r
 *
 * Input u_init:
 *   u1 = steering angle velocity of front wheels
 *   u2 = longitudinal acceleration
 *
 * Returns:
 *   f (size 29) = right-hand side of differential equations
 */
std::vector<double> vehicle_dynamics_mb(const std::vector<double>& x,
                                        const std::vector<double>& u_init,
                                        const VehicleParameters& p);

} // namespace velox::models
