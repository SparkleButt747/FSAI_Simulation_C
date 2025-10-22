#pragma once

#include <cstdint>
#include "DynamicBicycle.hpp"  // vehicle model
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "Telemetry.hpp"
#include "RacingAlgorithm.h" // from RacingAlgorithms
#include "Transform.h"       // from PhysicsEngine
#include "Vector.h"          // from PhysicsEngine
#include "KeyboardInputHandler.h" // for keyboard input handling

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

    // Track data: array of checkpoint positions and count.
    Vector3* checkpointPositions{nullptr};
    int nCheckpoints{0};
    // For lap detection: when the car "collides" with this checkpoint, a lap is complete.
    Vector3 lastCheckpoint{};

    // *** New: Cone data ***
    // Arrays for left and right cones and their counts.
    Vector3* leftCones{nullptr};
    int nLeftCones{0};
    Vector3* rightCones{nullptr};
    int nRightCones{0};

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
void CarController_Init(CarController* controller, const char* yamlFilePath);

// Update the simulation by dt seconds.
void CarController_Update(CarController* controller, double dt, uint64_t now_ns);

// Reset the car state and regenerate track.
void CarController_Reset(CarController* controller, uint64_t now_ns);

