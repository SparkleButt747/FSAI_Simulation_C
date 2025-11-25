#pragma once

#include <string>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * Creates a VehicleParameters object holding all vehicle parameters for
 * vehicle ID 4 (semi-trailer truck).
 */
inline VehicleParameters parameters_vehicle4(const std::string& dir_params = {})
{
    return setup_vehicle_parameters(4, dir_params);
}

} // namespace velox::models
