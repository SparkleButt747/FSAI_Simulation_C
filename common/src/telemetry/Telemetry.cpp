#include "Telemetry.hpp"
#include "fsai_clock.h"
#include <cstdio>

void Telemetry_Update(const VehicleState& carState, const Transform& carTransform,
                      uint64_t simTimeNs, double lapTimeSeconds,
                      double totalDistance, int lapCount) {
    const double simTimeSeconds = fsai_clock_to_seconds(simTimeNs);
    std::printf("Sim Time: %.3f s, Lap Time: %.2f s, Distance: %.2f, Lap: %d\n",
                simTimeSeconds, lapTimeSeconds, totalDistance, lapCount);
    std::printf("Car State -> x: %.2f, y: %.2f, yaw: %.2f, v_x: %.2f\n",
                carState.position.x(), carState.position.y(), carState.yaw, carState.velocity.x());
    std::printf("Car Transform -> Pos: (%.2f, %.2f, %.2f), Yaw: %.2f\n",
                carTransform.position.x, carTransform.position.y, carTransform.position.z, carTransform.yaw);
    std::printf("---------------------------------------------\n");
}
