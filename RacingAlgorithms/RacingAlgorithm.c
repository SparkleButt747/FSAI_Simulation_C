#include "RacingAlgorithm.h"
#include "math.h"
#include <stdlib.h>

// Helper function: Compute the car's forward direction (2D) from its yaw.
// We assume the car's forward direction in the horizontal plane is (cos(yaw), sin(yaw)).
static Vector2 GetCarForwardDirection(const Transform* transform) {
    Vector2 forward = { cosf(transform->yaw), sinf(transform->yaw) };
    return Vector2_Normalize(forward);
}

// Helper function: Clamp a float value between a minimum and maximum.
static float clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Helper function: Calculate the expected speed based on the angle (in degrees) and acceleration factor.
// If the angle is near zero, we assume full speed (MAX_SPEED). Otherwise, we use the expression:
// expectedSpeed = abs(1/(accelerationFactor * angle)), clamped to MAX_SPEED.
static float CalculateExpectedSpeed(float angle, float accelerationFactor) {
    if (fabsf(angle) < 0.001f) {
        return MAX_SPEED;
    }
    float expected = fabsf(1.0f / (accelerationFactor * angle));
    if (expected > MAX_SPEED) {
        expected = MAX_SPEED;
    }
    return expected;
}

// Helper function: Calculate the throttle input based on the acceleration needed.
// Uses positive throttle = accelerationNeeded/3.4323432343 and negative throttle = accelerationNeeded/10.0.
static float CalculateThrottle(float accelerationNeeded) {
    float posThrottle = 0.0f, negThrottle = 0.0f;
    if (accelerationNeeded > 0) {
        posThrottle = accelerationNeeded / 3.4323432343f;
    } else {
        negThrottle = accelerationNeeded / 10.0f; // accelerationNeeded is negative.
    }
    float throttle = posThrottle + negThrottle;
    return clamp_float(throttle, -1.0f, 1.0f);
}

// Computes throttle input based on checkpoint lookahead.
// This function calculates a lookahead index (based on speed sensitivity),
// then uses the checkpoint direction at that index to compute the desired speed
// and acceleration, and finally calculates the throttle value.
float RacingAlgorithm_GetThrottleInput(const Vector3* checkpointPositions, int nCheckpoints, 
                                         double carVelocity, const Transform* carTransform, 
                                         const RacingAlgorithmConfig* config, double dt) {
    if (nCheckpoints <= 0)
        return 0.0f;

    // Compute lookahead distances based on sensitivity.
    int speedLookaheadDistance = ((int)floorf(config->speedLookAheadSensitivity * (float)carVelocity)) + 1;
    int steeringLookaheadDistance = ((int)floorf(config->steeringLookAheadSensitivity * (float)carVelocity)) + 1;
    int lookaheadMax = (speedLookaheadDistance > steeringLookaheadDistance) ? speedLookaheadDistance : steeringLookaheadDistance;
    if (lookaheadMax > nCheckpoints - 1)
        lookaheadMax = nCheckpoints - 1;

    // Use variable-length arrays (VLAs) to cache checkpoint directions and distances.
    Vector2 cachedDirections[lookaheadMax + 1];
    float cachedDistances[lookaheadMax + 1];

    // Precompute for checkpoints [0, lookaheadMax].
    for (int i = 0; i <= lookaheadMax; i++) {
        Vector3 diff = { checkpointPositions[i].x - carTransform->position.x,
                         checkpointPositions[i].y - carTransform->position.y,
                         checkpointPositions[i].z - carTransform->position.z };
        // Consider only X and Z for horizontal motion.
        Vector2 diff2D = { diff.x, diff.z };
        float distance = Vector2_Magnitude(diff2D);
        cachedDistances[i] = distance;
        cachedDirections[i] = Vector2_Normalize(diff2D);
    }

    // Choose the lookahead index for throttle.
    int lookaheadIndex = (speedLookaheadDistance < (lookaheadMax + 1)) ? speedLookaheadDistance : lookaheadMax;

    Vector2 checkpointDir = cachedDirections[lookaheadIndex];
    float distanceToCheckpoint = cachedDistances[lookaheadIndex];

    Vector2 carForward = GetCarForwardDirection(carTransform);

    // Compute signed angle (in degrees) from car forward to checkpoint direction.
    float angle = Vector2_SignedAngle(carForward, checkpointDir);

    // Compute expected speed based on the angle.
    float expectedSpeed = CalculateExpectedSpeed(angle, config->accelerationFactor);

    // Compute acceleration needed to reach expected speed.
    float speed = (float)carVelocity;
    if (fabsf(speed) < 0.001f)
        speed = 0.1f; // Prevent division by zero.
    float accelerationNeeded = (expectedSpeed - speed) / (distanceToCheckpoint / speed);
    if (fabsf(accelerationNeeded) < 0.001f)
        accelerationNeeded = 1.0f;

    return CalculateThrottle(accelerationNeeded);
}

// Computes steering input based on checkpoint lookahead.
// This function uses the steering lookahead to determine the desired steering angle,
// calculates the rate at which the angle should change (based on reaction time),
// and returns the steering input (clamped between -1 and 1).
float RacingAlgorithm_GetSteeringInput(const Vector3* checkpointPositions, int nCheckpoints, 
                                       double carVelocity, const Transform* carTransform, 
                                       const RacingAlgorithmConfig* config, double dt) {
    if (nCheckpoints <= 0)
        return 0.0f;

    int speedLookaheadDistance = ((int)floorf(config->speedLookAheadSensitivity * (float)carVelocity)) + 1;
    int steeringLookaheadDistance = ((int)floorf(config->steeringLookAheadSensitivity * (float)carVelocity)) + 1;
    int lookaheadMax = (speedLookaheadDistance > steeringLookaheadDistance) ? speedLookaheadDistance : steeringLookaheadDistance;
    if (lookaheadMax > nCheckpoints - 1)
        lookaheadMax = nCheckpoints - 1;

    Vector2 cachedDirections[lookaheadMax + 1];
    float cachedDistances[lookaheadMax + 1];

    for (int i = 0; i <= lookaheadMax; i++) {
        Vector3 diff = { checkpointPositions[i].x - carTransform->position.x,
                         checkpointPositions[i].y - carTransform->position.y,
                         checkpointPositions[i].z - carTransform->position.z };
        Vector2 diff2D = { diff.x, diff.z };
        float distance = Vector2_Magnitude(diff2D);
        cachedDistances[i] = distance;
        cachedDirections[i] = Vector2_Normalize(diff2D);
    }

    int lookaheadIndex = (steeringLookaheadDistance < (lookaheadMax + 1)) ? steeringLookaheadDistance : lookaheadMax;

    Vector2 checkpointDir = cachedDirections[lookaheadIndex];
    float distanceToCheckpoint = cachedDistances[lookaheadIndex];

    Vector2 carForward = GetCarForwardDirection(carTransform);

    float angle = Vector2_SignedAngle(carForward, checkpointDir);

    float speed = (float)carVelocity;
    if (fabsf(speed) < 0.001f)
        speed = 0.1f;
    float reactionTime = distanceToCheckpoint / speed;
    if (fabsf(reactionTime) < 0.001f)
        reactionTime = 0.1f;

    float angleChangeRate = -angle / reactionTime;
    float updatedAngle = angle + angleChangeRate * (float)dt;

    float steeringInput = clamp_float(updatedAngle / 21.0f, -1.0f, 1.0f);
    return -steeringInput;
}
