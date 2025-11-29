#pragma once

#include <vector>

namespace velox::models {

struct VehicleParameters;

/**
 * init_std - generates the initial state vector for the drift single-track model.
 *
 * States:
 *  x1 = x-position in a global coordinate system
 *  x2 = y-position in a global coordinate system
 *  x3 = steering angle of front wheels
 *  x4 = velocity at vehicle center
 *  x5 = yaw angle
 *  x6 = yaw rate
 *  x7 = slip angle at vehicle center
 *  x8 = front wheel angular speed
 *  x9 = rear wheel angular speed
 */
std::vector<double> init_std(const std::vector<double>& init_state,
                             const VehicleParameters& p);

}  // namespace velox::models
