#pragma once

#include <string>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * Creates a VehicleParameters object holding all vehicle parameters for
 * vehicle ID 3 (VW Vanagon).
 */
inline VehicleParameters parameters_vehicle3(const std::string& dir_params = {})
{
    return setup_vehicle_parameters(3, dir_params);
}

} // namespace velox::models
