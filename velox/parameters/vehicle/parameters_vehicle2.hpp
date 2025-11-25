#pragma once

#include <string>
#include "vehicle_parameters.hpp"

namespace velox::models {

/**
 * Creates a VehicleParameters object holding all vehicle parameters for
 * vehicle ID 2 (BMW 320i).
 */
inline VehicleParameters parameters_vehicle2(const std::string& dir_params = {})
{
    return setup_vehicle_parameters(2, dir_params);
}

} // namespace velox::models
