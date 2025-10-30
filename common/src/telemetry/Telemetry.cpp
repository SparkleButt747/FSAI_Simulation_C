#include <common/telemetry/Telemetry.hpp>
#include <common/time/fsai_clock.h>
#include <cstdio>

void Telemetry_Update(const fsai::types::VehicleState& carState, const Transform& carTransform,
                      uint64_t simTimeNs, double lapTimeSeconds,
                      double totalDistance, int lapCount) {
    const double simTimeSeconds = fsai_clock_to_seconds(simTimeNs);
    std::printf("Run Time: %.3f s \n", simTimeSeconds);
}
