#include "Telemetry.h"
#include <stdio.h>

void Telemetry_Update(const State* carState, const Transform* carTransform,
                      double totalTime, double totalDistance, int lapCount) {
    printf("Time: %.2f s, Distance: %.2f, Lap: %d\n", totalTime, totalDistance, lapCount);
    printf("Car State -> x: %.2f, y: %.2f, yaw: %.2f, v_x: %.2f\n",
           carState->x, carState->y, carState->yaw, carState->v_x);
    printf("Car Transform -> Pos: (%.2f, %.2f, %.2f), Yaw: %.2f\n",
           carTransform->position.x, carTransform->position.y, carTransform->position.z, carTransform->yaw);
    printf("---------------------------------------------\n");
}
