#pragma once

#include <string>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * Creates a VehicleParameters object holding all vehicle parameters for
 * vehicle ID 1 (Ford Escort).
 */
inline VehicleParameters parameters_vehicle1(const std::string& dir_params = {})
{
    return setup_vehicle_parameters(1, dir_params);
}

} // namespace velox::models
