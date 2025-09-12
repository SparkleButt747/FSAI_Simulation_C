#include "Telemetry.hpp"
#include <cstdio>

void Telemetry_Update(const VehicleState& carState, const Transform& carTransform,
                      double totalTime, double totalDistance, int lapCount) {
    std::printf("Time: %.2f s, Distance: %.2f, Lap: %d\n", totalTime, totalDistance, lapCount);
    std::printf("Car State -> x: %.2f, y: %.2f, yaw: %.2f, v_x: %.2f\n",
                carState.position.x(), carState.position.y(), carState.yaw, carState.velocity.x());
    std::printf("Car Transform -> Pos: (%.2f, %.2f, %.2f), Yaw: %.2f\n",
                carTransform.position.x, carTransform.position.y, carTransform.position.z, carTransform.yaw);
    std::printf("---------------------------------------------\n");
}
