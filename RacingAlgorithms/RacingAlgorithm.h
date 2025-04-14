#ifndef RACINGALGORITHM_H
#define RACINGALGORITHM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Vector.h"
#include "Transform.h"

#define MAX_SPEED   30.1964649875f
#define MAX_ACC     3.4323432343f
#define MAX_ANGLE   21.0f
#define EPSILON     0.001f

    // Configuration parameters for the racing algorithm.
    typedef struct
    {
        float speedLookAheadSensitivity;    // e.g., 0.7
        float steeringLookAheadSensitivity; // e.g., 0.1
        float accelerationFactor;           // e.g., 0.002
    } RacingAlgorithmConfig;

    // Computes the throttle input (range -1 to 1) given the checkpoint data, current car velocity,
    // car transform, configuration, and simulation timestep dt.
    float RacingAlgorithm_GetThrottleInput(const Vector3 *checkpointPositions, int nCheckpoints,
                                           double carVelocity, const Transform *carTransform,
                                           const RacingAlgorithmConfig *config, double dt);

    // Computes the steering input (range -1 to 1) given the checkpoint data, current car velocity,
    // car transform, configuration, and simulation timestep dt.
    float RacingAlgorithm_GetSteeringInput(const Vector3 *checkpointPositions, int nCheckpoints,
                                           double carVelocity, const Transform *carTransform,
                                           const RacingAlgorithmConfig *config, double dt);

    LookahaedIndicies RacingAlgorithm_GetLookaheadIndices(int nCheckpoints, double carVelocity,
                                                const RacingAlgorithmConfig* config);

#ifdef __cplusplus
}
#endif

#endif // RACINGALGORITHM_H
