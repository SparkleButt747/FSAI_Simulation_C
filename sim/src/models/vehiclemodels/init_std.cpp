#include "models/vehiclemodels/init_std.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "vehicle_parameters.hpp"  // for VehicleParameters

namespace velox::models {

std::vector<double> init_std(const std::vector<double>& init_state,
                             const VehicleParameters& p)
{
    // Always start from a well-defined 7-state vector (position, heading,
    // steering, speed, yaw rate, slip angle). If the caller provides fewer
    // values, pad with zeros; if more, truncate to the expected base size so
    // the derived wheel speeds are computed consistently.
    constexpr std::size_t kBaseSize = 7;
    std::vector<double> x0(kBaseSize, 0.0);
    const auto copy = std::min(init_state.size(), x0.size());
    std::copy_n(init_state.begin(), copy, x0.begin());

    // x0[3] = velocity at vehicle center
    // x0[2] = steering angle (delta)
    // x0[6] = slip angle at vehicle center
    const double v     = x0[3];
    const double beta  = x0[6];
    const double delta = x0[2];

    // init front wheel angular speed
    x0.push_back(v * std::cos(beta) * std::cos(delta) / p.R_w);
    // init rear wheel angular speed
    x0.push_back(v * std::cos(beta) / p.R_w);

    return x0;
}

}  // namespace velox::models
