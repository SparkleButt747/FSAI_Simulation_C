#pragma once

#include <vector>
#include <Eigen/Dense>
#include "DynamicBicycle.hpp"
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "Telemetry.hpp"
#include "RacingAlgorithm.h"
#include "Transform.h"
#include "Vector.h"
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"

class World {
public:
    World() = default;

    // Initialize the world with vehicle parameters and generate track.
    void init(const char* yamlFilePath);

    // Update simulation by dt seconds.
    void update(double dt);

    // Detect collisions with checkpoints and cones. Returns false if a reset occurred.
    bool detectCollisions();

    // Output telemetry information.
    void telemetry() const;

    // Public control parameters for manual control or algorithms.
    float steeringAngle{0.0f};
    float throttleInput{0.0f};
    int useRacingAlgorithm{1};
    int regenTrack{1};

private:
    void moveNextCheckpointToLast();
    void reset();

    DynamicBicycle carModel{VehicleParam()};
    VehicleState carState{Eigen::Vector3d::Zero(), 0.0,
                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero()};
    VehicleInput carInput{0.0, 0.0, 0.0};
    Transform carTransform{};

    std::vector<Vector3> checkpointPositions{};
    std::vector<Vector3> leftCones{};
    std::vector<Vector3> rightCones{};
    Vector3 lastCheckpoint{0.0f, 0.0f, 0.0f};

    struct Config {
        float collisionThreshold{2.5f};
        float coneCollisionThreshold{0.5f};
        float lapCompletionThreshold{0.1f};
    } config{};

    RacingAlgorithmConfig racingConfig{};
    LookaheadIndices lookaheadIndices{};

    double totalTime{0.0};
    double deltaTime{0.0};
    double totalDistance{0.0};
    int lapCount{0};
};

