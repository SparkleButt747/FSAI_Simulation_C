#pragma once

#include <string>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * Creates a VehicleParameters object holding all vehicle parameters for
 * vehicle ID 5 (ADS-DV electric vehicle).
 */
inline VehicleParameters parameters_vehicle5(const std::string& dir_params = {})
{
    return setup_vehicle_parameters(5, dir_params);
}

} // namespace velox::models
