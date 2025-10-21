#include "control/racing_algorithm.h"
#include "math.h"
#include <stdlib.h>

// Helper function: Compute the car's forward direction (2D) from its yaw.
// We assume the car's forward direction in the horizontal plane is (cos(yaw), sin(yaw)).

static Vector2 GetCarForwardDirection(const Transform* transform) {
    Vector2 forward = { cosf(transform->yaw), sinf(transform->yaw) };
    return Vector2_Normalize(forward);
}

static float deg_to_rad(float deg) {
    return deg * 0.01745329251994329577f;
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
static float CalculateExpectedSpeed(float angle_deg, float accelerationFactor) {
    if (fabsf(angle_deg) < EPSILON) {
        return MAX_SPEED;
    }
    float expected = fabsf(1.0f / (accelerationFactor * angle_deg));
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
        posThrottle = accelerationNeeded / MAX_ACC;
    } else {
        negThrottle = accelerationNeeded / 10.0f; // accelerationNeeded is negative.
    }
    float throttle = posThrottle + negThrottle;
    return clamp_float(throttle, -1.0f, 1.0f);
}

LookaheadIndices RacingAlgorithm_GetLookaheadIndices(int nCheckpoints, double carVelocity,
                                                        const RacingAlgorithmConfig* config) {

    if (nCheckpoints <= 0) {
        LookaheadIndices indices = {-1, -1, -1};
        return indices;
    }

    int speedLookaheadDistance = ((int)floorf(config->speedLookAheadSensitivity * (float)carVelocity)) + 1;
    int steeringLookaheadDistance = ((int)floorf(config->steeringLookAheadSensitivity * (float)carVelocity)) + 1;

    if (speedLookaheadDistance < 1) speedLookaheadDistance = 1;
    if (steeringLookaheadDistance < 1) steeringLookaheadDistance = 1;

    int lookaheadMax = speedLookaheadDistance;
    if (steeringLookaheadDistance > lookaheadMax) {
        lookaheadMax = steeringLookaheadDistance;
    }
    if (lookaheadMax > nCheckpoints - 1) {
        lookaheadMax = nCheckpoints - 1;
    }

    int steeringIndex = steeringLookaheadDistance;
    if (steeringIndex > lookaheadMax) {
        steeringIndex = lookaheadMax;
    }
    int speedIndex = speedLookaheadDistance;
    if (speedIndex > lookaheadMax) {
        speedIndex = lookaheadMax;
    }

    LookaheadIndices indices = {steeringIndex, speedIndex, lookaheadMax};
    return indices;

}

// Computes throttle input based on checkpoint lookahead.
// This function calculates a lookahead index (based on speed sensitivity),
// then uses the checkpoint direction at that index to compute the desired speed
// and acceleration, and finally calculates the throttle value.
float RacingAlgorithm_GetThrottleInput(const Vector3* checkpointPositions, int nCheckpoints,
                                         double carVelocity, const Transform* carTransform,
                                         const RacingAlgorithmConfig* config, double dt) {

    (void)dt;

    // Choose the lookahead index for throttle.
    LookaheadIndices indices = RacingAlgorithm_GetLookaheadIndices(nCheckpoints, carVelocity, config);
    int lookaheadIndex = indices.speed;

    if (lookaheadIndex == -1) {
        return 0.0f;
    }
    Vector2 checkpointDir = {0.0f, 0.0f};
    float distanceToCheckpoint = 0.0f;
    for (int i = 0; i <= lookaheadIndex; ++i) {
        Vector3 diff = { checkpointPositions[i].x - carTransform->position.x,
                         checkpointPositions[i].y - carTransform->position.y,
                         checkpointPositions[i].z - carTransform->position.z };
        Vector2 diff2D = { diff.x, diff.z };
        float distance = Vector2_Magnitude(diff2D);
        if (distance <= EPSILON) {
            continue;
        }
        checkpointDir = Vector2_Normalize(diff2D);
        distanceToCheckpoint = distance;
    }

    if (distanceToCheckpoint <= EPSILON) {
        return 0.0f;
    }

    Vector2 carForward = GetCarForwardDirection(carTransform);

    // Compute signed angle (in degrees) from car forward to checkpoint direction.
    float angle_deg = Vector2_SignedAngle(carForward, checkpointDir);

    // Compute expected speed based on the angle.
    float expectedSpeed = CalculateExpectedSpeed(angle_deg, config->accelerationFactor);

    // Compute acceleration needed to reach expected speed.
    float speed = (float)carVelocity;
    if (fabsf(speed) < EPSILON) {
        speed = EPSILON;
    }
    float time_to_target = distanceToCheckpoint / speed;
    if (time_to_target < 0.1f) {
        time_to_target = 0.1f;
    }
    float accelerationNeeded = (expectedSpeed - speed) / time_to_target;

    return CalculateThrottle(accelerationNeeded);
}

// Computes steering input based on checkpoint lookahead.
// This function uses the steering lookahead to determine the desired steering angle,
// calculates the rate at which the angle should change (based on reaction time),
// and returns the steering input (clamped between -1 and 1).

float RacingAlgorithm_GetSteeringInput(
    const Vector3* checkpointPositions, int nCheckpoints,
    double carVelocity, const Transform* carTransform,
    const RacingAlgorithmConfig* config, double dt)
{
    LookaheadIndices indices =
        RacingAlgorithm_GetLookaheadIndices(nCheckpoints, carVelocity, config);
    int lookaheadIndex = indices.steer;

    if (lookaheadIndex == -1) {
        return 0.0f;
    }

    Vector2 checkpointDir = (Vector2){0.0f, 0.0f};
    float distanceToCheckpoint = 0.0f;
    for (int i = 0; i <= lookaheadIndex; ++i) {
        Vector3 diff = { checkpointPositions[i].x - carTransform->position.x,
                         checkpointPositions[i].y - carTransform->position.y,
                         checkpointPositions[i].z - carTransform->position.z };
        Vector2 diff2D = { diff.x, diff.z };
        float distance = Vector2_Magnitude(diff2D);
        if (distance <= EPSILON) {
            continue;
        }
        checkpointDir = Vector2_Normalize(diff2D);
        distanceToCheckpoint = distance;
    }

    if (distanceToCheckpoint <= EPSILON) {
        return 0.0f;
    }

    // Current forward & signed angle to checkpoint
    Vector2 carForward = GetCarForwardDirection(carTransform);
    float   angle_deg  = Vector2_SignedAngle(carForward, checkpointDir);
    float   angle_rad  = deg_to_rad(angle_deg);

    // —— Reaction-time smoothing block ——
    float speed = (float)carVelocity;
    if (fabsf(speed) < EPSILON) {
        speed = EPSILON;  // avoid div0
    }
    float reactionTime = distanceToCheckpoint / speed;

    // clamp reactionTime into [0.1…1.0]
    if (reactionTime < 0.001f || speed < 0.01f) {
        reactionTime = 0.1f;
    }
    if (reactionTime > 1.0f) {
        reactionTime = 1.0f;
    }

    // compute how quickly we need to steer to zero the angle in that window
    float angleChangeRate = -angle_rad / reactionTime;
    float updatedAngle    = angle_rad + angleChangeRate * (float)dt;

    if (updatedAngle > MAX_STEER_RAD) {
        updatedAngle = MAX_STEER_RAD;
    } else if (updatedAngle < -MAX_STEER_RAD) {
        updatedAngle = -MAX_STEER_RAD;
    }

    return updatedAngle;
}

