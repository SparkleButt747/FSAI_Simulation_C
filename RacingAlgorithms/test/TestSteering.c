#include <stdio.h>
#include "RacingAlgorithm.h"
#include "Vector.h"

int main(void) {
    Vector3 checkpoints[] = {
        {3.92, 0.00, 0.19},
        {7.36, 0.00, 0.83},
        {10.04, 0.00, 1.96},
        {11.73, 0.00, 3.56},
        {12.31, 0.00, 5.52},
        {11.79, 0.00, 7.70},
        {10.27, 0.00, 9.96},
        {7.95, 0.00, 12.17},
        {5.10, 0.00, 14.24},
        {2.02, 0.00, 16.12},
        {-1.04, 0.00, 17.82},
        {-3.84, 0.00, 19.40},
        {-6.24, 0.00, 20.93},
        {-8.17, 0.00, 22.47},
        {-9.67, 0.00, 24.10},
        {-10.81, 0.00, 25.83},
        {-11.75, 0.00, 27.65},
        {-12.64, 0.00, 29.51}};
    int nCheckpoints = 12;
    double carVelocity = 0;
    Transform transform = {{0, 0, 0}, 0};
    RacingAlgorithmConfig config = {0.7, 0.1, 0.002};
    double dt = 0.1;

    float throttle1 = RacingAlgorithm_GetSteeringInput(
        checkpoints, nCheckpoints,
        carVelocity, &transform,
        &config, dt
    );
    printf("%f", throttle1);
    // (const Vector3* checkpointPositions, int nCheckpoints, 
        // double carVelocity, const Transform* carTransform, 
        // const RacingAlgorithmConfig* config, double dt)
    return 0;
}
