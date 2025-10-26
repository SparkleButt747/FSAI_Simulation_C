#pragma once

#include <vector>
#include <Eigen/Dense>
#include "DynamicBicycle.hpp"
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "Telemetry.hpp"
#include "controller.h"
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
    float brakeInput{0.0f};
    int useController{1};
    int regenTrack{1};

    const VehicleState& vehicleState() const { return carState; }
    const Transform& vehicleTransform() const { return carTransform; }
    const std::vector<Vector3>& checkpointPositionsWorld() const {
        return checkpointPositions;
    }
    const std::vector<Vector3>& leftConePositions() const { return leftCones; }
    const std::vector<Vector3>& rightConePositions() const { return rightCones; }
    const LookaheadIndices& lookahead() const { return lookaheadIndices; }
    const WheelsInfo& wheelsInfo() const { return wheelsInfo_; }
    double lapTimeSeconds() const { return totalTime; }
    double totalDistanceMeters() const { return totalDistance; }
    double timeStepSeconds() const { return deltaTime; }
    int completedLaps() const { return lapCount; }

    bool computeRacingControl(double dt, float& throttle_out, float& steering_out);
    void setSvcuCommand(float throttle, float brake, float steer);
    bool hasSvcuCommand() const { return hasSvcuCommand_; }
    const DynamicBicycle& model() const { return carModel; }

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

    WheelsInfo wheelsInfo_{WheelsInfo_default()};
    bool hasSvcuCommand_{false};
    float lastSvcuThrottle_{0.0f};
    float lastSvcuBrake_{0.0f};
    float lastSvcuSteer_{0.0f};

    struct Config {
        float collisionThreshold{2.5f};
        float coneCollisionThreshold{0.5f};
        float lapCompletionThreshold{0.1f};
    } config{};

    ControllerConfig racingConfig{};
    LookaheadIndices lookaheadIndices{};

    double totalTime{0.0};
    double deltaTime{0.0};
    double totalDistance{0.0};
    int lapCount{0};
};

