#pragma once

#include <cstdint>
#include "VehicleState.hpp"
#include "Transform.h"

// Prints simulation telemetry to the terminal.
void Telemetry_Update(const VehicleState& carState, const Transform& carTransform,
                      uint64_t simTimeNs, double lapTimeSeconds,
                      double totalDistance, int lapCount);

