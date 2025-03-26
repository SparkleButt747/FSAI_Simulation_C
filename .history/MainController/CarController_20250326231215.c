#include "CarController.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Assume track_generator.h defines:
//   void init_default_path_config(PathConfig* config);
//   PathResult generate_path_with_params(const PathConfig* config, int nPoints);
//   TrackResult generate_track(const PathConfig* config, const PathResult* path);

#include "track_generator.h"

// Initialize CarController.
void CarController_Init(CarController* controller, const char* yamlFilePath) {
    // Initialize simulation variables.
    controller->totalTime = 0.0;
    controller->totalDistance = 0.0;
    controller->lapCount = 0;
    controller->deltaTime = 0.0;

    // Set controller configuration.
    controller->config.collisionThreshold = 1.0f;  // example threshold

    // Use racing algorithm by default.
    controller->useRacingAlgorithm = 1;

    // Initialize RacingAlgorithm configuration.
    controller->racingConfig.speedLookAheadSensitivity = 0.7f;
    controller->racingConfig.steeringLookAheadSensitivity = 0.1f;
    controller->racingConfig.accelerationFactor = 0.002f;

    // Initialize car model, state, and input.
    DynamicBicycle_init(&controller->carModel, yamlFilePath);
    controller->carInput = Input_create(0.0, 0.0, 0.0);
    controller->carState = State_create(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Initialize car transform based on the state.
    controller->carTransform.position.x = (float)controller->carState.x;
    controller->carTransform.position.y = 0.5f;  // fixed height
    controller->carTransform.position.z = (float)controller->carState.y;  // map state.y to z
    controller->carTransform.yaw = (float)controller->carState.yaw;


    // Generate track data.
    PathConfig pathConfig;
    init_default_path_config(&pathConfig);
    int nPoints = pathConfig.resolution;
    PathResult path = generate_path_with_params(&pathConfig, nPoints);
    TrackResult track = generate_track(&pathConfig, &path);

    // Store checkpoint data.
    controller->nCheckpoints = track.nCheckpoints;
    controller->checkpointPositions = (Vector3*)malloc(sizeof(Vector3) * controller->nCheckpoints);
    for (int i = 0; i < controller->nCheckpoints; i++) {
        controller->checkpointPositions[i] = track.checkpoints[i].position;
    }
    // For lap detection, use the last checkpoint in the array.
    if (controller->nCheckpoints > 0) {
        controller->lastCheckpoint = track.checkpoints[controller->nCheckpoints - 1].position;
    } else {
        controller->lastCheckpoint = (Vector3){0, 0, 0};
    }

    // *** New: Store cone data ***
    // Left cones.
    controller->nLeftCones = track.nLeftCones;
    controller->leftCones = (Vector3*)malloc(sizeof(Vector3) * controller->nLeftCones);
    for (int i = 0; i < controller->nLeftCones; i++) {
        controller->leftCones[i] = track.leftCones[i].position;
    }
    // Right cones.
    controller->nRightCones = track.nRightCones;
    controller->rightCones = (Vector3*)malloc(sizeof(Vector3) * controller->nRightCones);
    for (int i = 0; i < controller->nRightCones; i++) {
        controller->rightCones[i] = track.rightCones[i].position;
    }

    // Free temporary track data.
    free(path.points);
    free(path.normals);
    free(path.cornerRadii);
    free(track.leftCones);
    free(track.rightCones);
    free(track.checkpoints);
}

// Update simulation.
void CarController_Update(CarController* controller, double dt) {
    // Update simulation time.
    controller->deltaTime = dt;
    controller->totalTime += dt;

    // Validate that there are checkpoints.
    if (controller->nCheckpoints <= 0) {
        printf("No checkpoints available. Resetting simulation.\n");
        CarController_Reset(controller);
        return;
    }

    // Handle input.
    if (controller->useRacingAlgorithm) {
        controller->throttleInput = RacingAlgorithm_GetThrottleInput(
            controller->checkpointPositions,
            controller->nCheckpoints,
            controller->carState.v_x,
            &controller->carTransform,
            &controller->racingConfig,
            dt);
        controller->steeringAngle = RacingAlgorithm_GetSteeringInput(
            controller->checkpointPositions,
            controller->nCheckpoints,
            controller->carState.v_x,
            &controller->carTransform,
            &controller->racingConfig,
            dt);
    } else {
        printf("Manual control mode. Use arrow keys to control the car.\n");
        // Read keyboard input.
        int key = KeyboardInputHandler_GetInput();
        // Use keyboard input for control.
        if (key != -1) {
            if (key == 'w' || key == 'W') {
                controller->throttleInput = 1.0;  // Accelerate
            } else if (key == 's' || key == 'S') {
                controller->throttleInput = -1.0; // Decelerate
            }
            if (key == 'a' || key == 'A') {
                controller->steeringAngle = -1.0; // Full left
            } else if (key == 'd' || key == 'D') {
                controller->steeringAngle= 1.0;  // Full right
            }
        }
    }

    // Update car input.
    controller->carInput.acc = controller->throttleInput;
    controller->carInput.delta = controller->steeringAngle;

    // Update car state.
    DynamicBicycle_UpdateState(&controller->carModel, &controller->carState, &controller->carInput, dt);

    // Update transform from car state.
    controller->carTransform.position.x = (float)controller->carState.x;
    controller->carTransform.position.z = (float)controller->carState.y;  // map state.y to z coordinate
    controller->carTransform.yaw = (float)controller->carState.yaw;

    // Update total distance traveled (using v_x).
    controller->totalDistance += controller->carState.v_x * dt;

    // Lap detection: Check if car "collides" with the last checkpoint.
    float dx = controller->carTransform.position.x - controller->lastCheckpoint.x;
    float dz = controller->carTransform.position.z - controller->lastCheckpoint.z;
    float distToLast = sqrtf(dx * dx + dz * dz);
    if (distToLast < controller->config.collisionThreshold) {
        // Lap completed.
        if (controller->lapCount > 0) {
            printf("Lap Completed. Time: %.2f s, Distance: %.2f, Lap: %d\n",
                   controller->totalTime, controller->totalDistance, controller->lapCount);
        }
        controller->lapCount++;
        // Reset timing and distance for next lap.
        controller->totalTime = 0.0;
        controller->totalDistance = 0.0;
    }

    // Telemetry: Print current simulation state.
    Telemetry_Update(&controller->carState, &controller->carTransform,
                     controller->totalTime, controller->totalDistance, controller->lapCount);
}

// Reset simulation: Reset car state and regenerate track.
void CarController_Reset(CarController* controller) {
    // Reset car state, input, and transform.
    controller->carInput = Input_create(0.0, 0.0, 0.0);
    controller->carState = State_create(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    controller->carTransform.position.x = 0.0f;
    controller->carTransform.position.y = 0.5f;
    controller->carTransform.position.z = 0.0f;
    controller->carTransform.yaw = 0.0f;

    // Regenerate track.
    PathConfig pathConfig;
    init_default_path_config(&pathConfig);
    int nPoints = pathConfig.resolution;
    PathResult path = generate_path_with_params(&pathConfig, nPoints);
    TrackResult track = generate_track(&pathConfig, &path);

    // Free old checkpoint data.
    if (controller->checkpointPositions != NULL) {
        free(controller->checkpointPositions);
    }
    controller->nCheckpoints = track.nCheckpoints;
    controller->checkpointPositions = (Vector3*)malloc(sizeof(Vector3) * controller->nCheckpoints);
    for (int i = 0; i < controller->nCheckpoints; i++) {
        controller->checkpointPositions[i] = track.checkpoints[i].position;
    }
    if (controller->nCheckpoints > 0) {
        controller->lastCheckpoint = track.checkpoints[controller->nCheckpoints - 1].position;
    } else {
        controller->lastCheckpoint = (Vector3){0, 0, 0};
    }

    // *** Update cone data as well ***
    // Free old cone data if any.
    if (controller->leftCones != NULL) {
        free(controller->leftCones);
    }
    if (controller->rightCones != NULL) {
        free(controller->rightCones);
    }
    controller->nLeftCones = track.nLeftCones;
    controller->nRightCones = track.nRightCones;
    controller->leftCones = (Vector3*)malloc(sizeof(Vector3) * controller->nLeftCones);
    for (int i = 0; i < controller->nLeftCones; i++) {
        controller->leftCones[i] = track.leftCones[i].position;
    }
    controller->rightCones = (Vector3*)malloc(sizeof(Vector3) * controller->nRightCones);
    for (int i = 0; i < controller->nRightCones; i++) {
        controller->rightCones[i] = track.rightCones[i].position;
    }

    controller->totalTime = 0.0;
    controller->totalDistance = 0.0;

    free(path.points);
    free(path.normals);
    free(path.cornerRadii);
    free(track.leftCones);
    free(track.rightCones);
    free(track.checkpoints);
}
