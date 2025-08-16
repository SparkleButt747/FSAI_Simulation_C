#include "CarController.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Assume track_generator.h defines:
//   void init_default_path_config(PathConfig* config);
//   PathResult generate_path_with_params(const PathConfig* config, int nPoints);
//   TrackResult generate_track(const PathConfig* config, const PathResult* path);

#include "track_generator.h"

static void MoveNextCheckpointToLast(Vector3* checkpoints, Vector3* lefts, Vector3* rights,int n) {
    Vector3 temp = checkpoints[0];
    for (int i = 1; i < n; i ++) {
        checkpoints[i-1] = checkpoints[i];
    }
    checkpoints[n-1] = temp;
    
    temp = lefts[0];
    for (int i = 1; i < n; i ++) {
        lefts[i-1] = lefts[i];
    }
    lefts[n-1] = temp;

    temp = rights[0];
    for (int i = 1; i < n; i ++) {
        rights[i-1] = rights[i];
    }
    rights[n-1] = temp;
};

// Initialize CarController.
void CarController_Init(CarController* controller, const char* yamlFilePath) {

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

    // Initialize simulation variables.
    controller->totalTime = 0.0;
    controller->totalDistance = 0.0;
    controller->lapCount = 0;
    controller->deltaTime = 0.0;

    // Set controller configuration.
    controller->config.collisionThreshold = 2.5f;  //  
    controller->config.conecollisionThreshold = 0.5f;  // 
    controller->config.lap_completion_collisionThreshold = 0.1f;  // crude implementation for now;

    // Use racing algorithm by default.
    controller->useRacingAlgorithm = 1;

    // Regenerate track after cone collision.
    controller->regenTrack = 1;

    // Initialize RacingAlgorithm configuration.
    controller->racingConfig.speedLookAheadSensitivity = 0.7f;
    controller->racingConfig.steeringLookAheadSensitivity = 0.1f;
    controller->racingConfig.accelerationFactor = 0.002f;

    // Make sure the car starts on the track
    float startX = controller->checkpointPositions[0].x;
    float startZ = controller->checkpointPositions[0].z;

    // Make sure the car is pointing in the right direction
    Vector2 zeroVector = {0, 0};
    float nextX = controller->checkpointPositions[10].x;
    float nextZ = controller->checkpointPositions[10].z;
    Vector2 startVector = {nextX - startX, nextZ - startZ};
    float startYaw = Vector2_SignedAngle(zeroVector, startVector) * M_PI/180;

    // Initialize car model, state, and input.
    DynamicBicycle_init(&controller->carModel, yamlFilePath);
    controller->carInput = Input_create(0.0, 0.0, 0.0);
    controller->carState = State_create(startX, startZ, 0, startYaw, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Initialize car transform based on the state.
    controller->carTransform.position.x = (float)controller->carState.x;
    controller->carTransform.position.y = 0.5f;  // fixed height
    controller->carTransform.position.z = (float)controller->carState.y;  // map state.y to z
    controller->carTransform.yaw = (float)controller->carState.yaw;

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

    // Initialize keyboard input.
    KeyboardInputHandler_Init();
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
        printf("Racing algorithm mode. Using racing algorithm for control.\n");
        Vector3 carVelocity = {controller->carState.v_x, controller->carState.v_y, controller->carState.v_z};
        float carSpeed = Vector3_Magnitude(carVelocity);
        controller->lookaheadIndices = RacingAlgorithm_GetLookaheadIndices(
            controller->nCheckpoints,
            carSpeed,
            &controller->racingConfig);
        controller->throttleInput = RacingAlgorithm_GetThrottleInput(
            controller->checkpointPositions,
            controller->nCheckpoints,
            carSpeed,
            &controller->carTransform,
            &controller->racingConfig,
            dt);
        controller->steeringAngle = RacingAlgorithm_GetSteeringInput(
            controller->checkpointPositions,
            controller->nCheckpoints,
            carSpeed,
            &controller->carTransform,
            &controller->racingConfig,
            dt);
        printf("THROTTLE: %f, ANGLE: %f\n", controller->throttleInput, controller->steeringAngle);
    } else {
        printf("Manual control mode. Use WASD keys to control the car.\n");
        // Read keyboard input.
        int key = KeyboardInputHandler_GetInput();
        printf("%c", key);
        if (key != -1) {
            if (key == 'w' || key == 'W') {
                controller->throttleInput = 1.0;  // Accelerate
                printf("Accelerating\n");
            } else if (key == 's' || key == 'S') {
                controller->throttleInput = -1.0; // Decelerate
                printf("Decelerating\n");
            }
            if (key == 'a' || key == 'A') {
                controller->steeringAngle = -1.0; // Full left
                printf("Turning left\n");
            } else if (key == 'd' || key == 'D') {
                controller->steeringAngle= 1.0;  // Full right
                printf("Turning right\n");
            }
        }
        printf("THROTTLE: %f, ANGLE: %f\n", controller->throttleInput, controller->steeringAngle);
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

    // Checkpoint met detection: Check if car passes its next checkpoint.
    float dx = controller->carTransform.position.x - controller->checkpointPositions[0].x;
    float dz = controller->carTransform.position.z - controller->checkpointPositions[0].z;
    float distToNext = sqrtf(dx * dx + dz * dz);
    if (distToNext < controller->config.collisionThreshold) {
        MoveNextCheckpointToLast(controller->checkpointPositions, controller->leftCones,
            controller->rightCones, controller->nCheckpoints);
    }
    printf("Next Checkpoint: (%f, %f)\n", controller->checkpointPositions[0].x, controller->checkpointPositions[0].z);


    // Lap detection: Check if car "collides" with the last checkpoint.
    dx = controller->carTransform.position.x - controller->lastCheckpoint.x;
    dz = controller->carTransform.position.z - controller->lastCheckpoint.z;
    float distToLast = sqrtf(dx * dx + dz * dz);
    if (distToLast < controller->config.lap_completion_collisionThreshold) {
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

    // Cone collision detection.
    for (int i = 0; i < controller->nLeftCones; i++) {
        float cdx = controller->carTransform.position.x - controller->leftCones[i].x;
        float cdz = controller->carTransform.position.z - controller->leftCones[i].z;
        float cdist = sqrtf(cdx * cdx + cdz * cdz);
        if (cdist < controller->config.conecollisionThreshold) {
            printf("Collision with a cone detected.\n");
            CarController_Reset(controller);
            return;
        }
    }
    for (int i = 0; i < controller->nRightCones; i++) {
        float cdx = controller->carTransform.position.x - controller->rightCones[i].x;
        float cdz = controller->carTransform.position.z - controller->rightCones[i].z;
        float cdist = sqrtf(cdx * cdx + cdz * cdz);
        if (cdist < controller->config.conecollisionThreshold) {
            printf("Collision with a cone detected.\n");
            CarController_Reset(controller);
            return;
        }
    }

    // Telemetry: Print current simulation state.
    Telemetry_Update(&controller->carState, &controller->carTransform,
                     controller->totalTime, controller->totalDistance, controller->lapCount);
}

// Reset simulation: Reset car state and regenerate track.
void CarController_Reset(CarController* controller) {

    if (controller->regenTrack) {

        controller->totalTime = 0.0;
        controller->totalDistance = 0.0;

        printf("Regenerating track due to cone collision.\n");

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

         // Reset car state, input, and transform.
        controller->carInput = Input_create(0.0, 0.0, 0.0);

        // Make sure the car starts on the track
        float startX = controller->checkpointPositions[0].x;
        float startZ = controller->checkpointPositions[0].z;
            
        // Make sure the car is pointing in the right direction
        Vector2 zeroVector = {0, 0};
        float nextX = controller->checkpointPositions[10].x;
        float nextZ = controller->checkpointPositions[10].z;
        Vector2 startVector = {nextX - startX, nextZ - startZ};
        float startYaw = Vector2_SignedAngle(zeroVector, startVector) * M_PI/180;

        controller->carState = State_create(startX, startZ, 0, startYaw, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        // Initialize car transform based on the state.
        controller->carTransform.position.x = (float)controller->carState.x;
        controller->carTransform.position.y = 0.5f;  // fixed height
        controller->carTransform.position.z = (float)controller->carState.y;  // map state.y to z
        controller->carTransform.yaw = (float)controller->carState.yaw;

        free(path.points);
        free(path.normals);
        free(path.cornerRadii);
        free(track.leftCones);
        free(track.rightCones);
        free(track.checkpoints);

    } else {
        // Reset car state, input, and transform.
        controller->carInput = Input_create(0.0, 0.0, 0.0);

        controller->totalTime = 0.0;
        controller->totalDistance = 0.0;

        printf("Resetting simulation without regenerating track.\n");

        // Make sure the car starts on the track
        float startX = controller->checkpointPositions[0].x;
        float startZ = controller->checkpointPositions[0].z;
            
        // Make sure the car is pointing in the right direction
        Vector2 zeroVector = {0, 0};
        float nextX = controller->checkpointPositions[10].x;
        float nextZ = controller->checkpointPositions[10].z;
        Vector2 startVector = {nextX - startX, nextZ - startZ};
        float startYaw = Vector2_SignedAngle(zeroVector, startVector) * M_PI/180;

        controller->carState = State_create(startX, startZ, 0, startYaw, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        // Initialize car transform based on the state.
        controller->carTransform.position.x = (float)controller->carState.x;
        controller->carTransform.position.y = 0.5f;  // fixed height
        controller->carTransform.position.z = (float)controller->carState.y;  // map state.y to z
        controller->carTransform.yaw = (float)controller->carState.yaw;
    }
}
