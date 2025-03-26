#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "DynamicBicycle.h"  // from SimulationModels/DynamicBicycle
#include "RacingAlgorithm.h" // from RacingAlgorithms
#include "Telemetry.h"       // from Telemetry
#include "track_generator.h" // provides PathConfig, PathResult, TrackResult, and track generation functions
#include "Transform.h"       // from PhysicsEngine
#include "Vector.h"          // from PhysicsEngine
#include "KeyboardInputHandler.h" // new: for keyboard input handling

// Configuration parameters for CarController.
typedef struct
{
    float collisionThreshold; // Distance threshold for collision detection.
} CarControllerConfig;

// Main CarController structure.
typedef struct
{
    // Car physics model (dynamic bicycle), state, and input.
    DynamicBicycle carModel;
    State carState;
    Input carInput;

    // Simulated transform (position and yaw).
    Transform carTransform;

    // Racing algorithm configuration.
    RacingAlgorithmConfig racingConfig;

    // Track data: array of checkpoint positions and count.
    Vector3 *checkpointPositions;
    int nCheckpoints;
    // For lap detection: when the car "collides" with this checkpoint, a lap is complete.
    Vector3 lastCheckpoint;

    // *** New: Cone data ***
    // Arrays for left and right cones and their counts.
    Vector3 *leftCones;
    int nLeftCones;
    Vector3 *rightCones;
    int nRightCones;

    // Simulation timing and distance tracking.
    double totalTime;
    double deltaTime;
    double totalDistance;
    int lapCount;

    // Controller configuration.
    CarControllerConfig config;

    // Control outputs.
    float steeringAngle;
    float throttleInput;

    // Flag: use racing algorithm (1) or manual input (0).
    int useRacingAlgorithm;

} CarController;

// Initialize the CarController: load car model, set initial state, and generate track.
void CarController_Init(CarController *controller, const char *yamlFilePath);

// Update the simulation by dt seconds.
void CarController_Update(CarController *controller, double dt);

// Reset the car state and regenerate track.
void CarController_Reset(CarController *controller);

#ifdef __cplusplus
}
#endif

#endif // CARCONTROLLER_H
