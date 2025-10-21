#pragma once

#include <cstdint>
#include "DynamicBicycle.hpp"  // vehicle model
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "Telemetry.hpp"
#include "control/racing_algorithm.h"
#include "Transform.h"       // from PhysicsEngine
#include "Vector.h"          // from PhysicsEngine
#include "common/types.h"
#include "sim/integration/path_truth.hpp"

struct CarControllerConfig {
    float collisionThreshold; // Distance threshold for collision detection.
    float conecollisionThreshold; // Distance threshold for cone collision detection.
    float lap_completion_collisionThreshold; // Distance threshold for lap completion detection.
};

struct CarController {
    // Car physics model (dynamic bicycle), state, and input.
    DynamicBicycle carModel;
    VehicleState carState;
    VehicleInput carInput;

    // Simulated transform (position and yaw).
    Transform carTransform;

    // Racing algorithm configuration.
    RacingAlgorithmConfig racingConfig;

    // Track data supplied by planner/adapter.
    const Vector3* checkpointPositions{nullptr};
    int nCheckpoints{0};
    // Reference to generated truth for visualization/reference.
    const fsai::integration::PathTruth* pathTruth{nullptr};

    // Simulation timing and distance tracking.
    double totalTime{0.0};
    double deltaTime{0.0};
    uint64_t startTimeNs{0};
    uint64_t lapStartTimeNs{0};
    uint64_t lastTickNs{0};
    double totalDistance{0.0};
    int lapCount{0};

    // Controller configuration.
    CarControllerConfig config{};

    // Control outputs.
    float steeringAngle{0.0f};
    float throttleInput{0.0f};

    // Flag: use racing algorithm (1) or manual input (0).
    int useRacingAlgorithm{1};

    // Flag: Regenerate Track (1) or Use Same Track (0).
    int regenTrack{1};

    LookaheadIndices lookaheadIndices{};
};

// Initialize the CarController: load car model, set initial state, and generate track.
void CarController_Init(CarController* controller,
                       const char* yamlFilePath,
                       const fsai::integration::PathTruth* truth);

void CarController_SetCheckpoints(CarController* controller,
                                  const Vector3* checkpoints,
                                  int count);

// Update the simulation by dt seconds.
FsaiControlCmd CarController_Update(CarController* controller, double dt, uint64_t now_ns);

// Reset the car state and regenerate track.
void CarController_Reset(CarController* controller, uint64_t now_ns);

