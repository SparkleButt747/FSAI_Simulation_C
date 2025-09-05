#pragma once

#include "VehicleState.hpp"
#include "Transform.h"

// Prints simulation telemetry to the terminal.
void Telemetry_Update(const VehicleState& carState, const Transform& carTransform,
                      double totalTime, double totalDistance, int lapCount);

