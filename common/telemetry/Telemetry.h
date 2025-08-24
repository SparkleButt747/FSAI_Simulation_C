#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "State.h"
#include "Transform.h"

// Prints simulation telemetry to the terminal.
void Telemetry_Update(const State* carState, const Transform* carTransform,
                      double totalTime, double totalDistance, int lapCount);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_H
