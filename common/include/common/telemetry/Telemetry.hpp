#pragma once

#include <cstdint>

#include <common/math/Transform.h>
#include <common/types.h>

// Prints simulation telemetry to the terminal.
void Telemetry_Update(const fsai::types::VehicleState& carState, const Transform& carTransform,
                      uint64_t simTimeNs, double lapTimeSeconds,
                      double totalDistance, int lapCount);

