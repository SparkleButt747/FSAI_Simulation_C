#pragma once

#include <vector>
#include <Eigen/Dense>
#include "types.h"
#include "DynamicBicycle.hpp"
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "Telemetry.hpp"
#include "controller.prot.h"
#include "Transform.h"
#include "Vector.h"
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "sim/mission/MissionDefinition.hpp"
#include "sim/MissionRuntimeState.hpp"

enum class ConeType {
    Start,
    Left,
    Right,
};

struct Cone {
    Vector3 position{0.0f, 0.0f, 0.0f};
    float radius{0.0f};
    float mass{0.0f};
    ConeType type{ConeType::Left};
};

struct CollisionSegment {
    Vector2 start{0.0f, 0.0f};
    Vector2 end{0.0f, 0.0f};
    float radius{0.0f};
    Vector2 boundsMin{0.0f, 0.0f};
    Vector2 boundsMax{0.0f, 0.0f};
};

class World {
public:
    World() = default;

    // Initialize the world with vehicle parameters and generate track.
    void init(const char* yamlFilePath, fsai::sim::MissionDefinition mission);

    // Update simulation by dt seconds.
    void update(double dt);

    // Detect collisions with checkpoints and cones. Returns false if a reset occurred.
    bool detectCollisions(bool crossedGate);

    // Output telemetry information.
    void telemetry() const;

    // Public control parameters for manual control or algorithms.
    float steeringAngle{0.0f};
    float throttleInput{0.0f};
    float brakeInput{0.0f};
    int useController{1};
    int regenTrack{1};
    std::vector<std::pair<Vector2, Vector2>> bestPathEdges {};
    std::vector<FsaiConeDet> coneDetections {};


    const VehicleState& vehicleState() const { return carState; }
    const Transform& vehicleTransform() const { return carTransform; }
    const std::vector<Vector3>& checkpointPositionsWorld() const {
        return checkpointPositions;
    }

    const std::vector<Cone>& getStartCones() const { return startCones; }
    const std::vector<Cone>& getLeftCones() const { return leftCones; }
    const std::vector<Cone>& getRightCones() const { return rightCones; }    
    const std::vector<Vector3> getStartConePositions() const {
        std::vector<Vector3> positions;
        positions.reserve(startCones.size());
        for (auto c: startCones) {
            positions.push_back(c.position);
        }
        return positions;
    }
    const std::vector<Vector3> getLeftConePositions() const {
        std::vector<Vector3> positions;
        positions.reserve(leftCones.size());
        for (auto c: leftCones) {
            positions.push_back(c.position);
        }
        return positions;
    }
    const std::vector<Vector3> getRightConePositions() const {
        std::vector<Vector3> positions;
        positions.reserve(rightCones.size());
        for (auto c: rightCones) {
            positions.push_back(c.position);
        }
        return positions;
    }
    const LookaheadIndices& lookahead() const { return lookaheadIndices; }
    const WheelsInfo& wheelsInfo() const { return wheelsInfo_; }
    double lapTimeSeconds() const { return totalTime; }
    double totalDistanceMeters() const { return totalDistance; }
    double timeStepSeconds() const { return deltaTime; }
    int completedLaps() const { return lapCount; }
    double missionElapsedSeconds() const { return missionState_.mission_time_seconds(); }
    double straightLineProgressMeters() const { return missionState_.straight_line_progress_m(); }
    fsai::sim::MissionRunStatus missionRunStatus() const { return missionState_.run_status(); }
    const fsai::sim::MissionRuntimeState& missionRuntime() const { return missionState_; }

    bool computeRacingControl(double dt, float& throttle_out, float& steering_out);
    void setSvcuCommand(float throttle, float brake, float steer);
    bool hasSvcuCommand() const { return hasSvcuCommand_; }
    const DynamicBicycle& model() const { return carModel; }

    const fsai::sim::MissionDefinition& mission() const { return mission_; }

private:
    friend class WorldTestHelper;
    void moveNextCheckpointToLast();
    void reset();
    void configureTrackState(const fsai::sim::TrackData& track);
    void initializeVehiclePose();
    fsai::sim::TrackData generateRandomTrack() const;

    DynamicBicycle carModel{VehicleParam()};
    VehicleState carState{Eigen::Vector3d::Zero(), 0.0,
                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero()};
    VehicleInput carInput{0.0, 0.0, 0.0};
    Transform carTransform{};
    Vector2 prevCarPos_{0.0f, 0.0f};

    std::vector<Vector3> checkpointPositions{};
    std::vector<Cone> startCones{};
    std::vector<Cone> leftCones{};
    std::vector<Cone> rightCones{};
    std::vector<CollisionSegment> gateSegments_{};
    std::vector<CollisionSegment> boundarySegments_{};
    Vector3 lastCheckpoint{0.0f, 0.0f, 0.0f};

    WheelsInfo wheelsInfo_{WheelsInfo_default()};
    bool hasSvcuCommand_{false};
    float lastSvcuThrottle_{0.0f};
    float lastSvcuBrake_{0.0f};
    float lastSvcuSteer_{0.0f};

    struct Config {
        float collisionThreshold{2.5f};
        float vehicleCollisionRadius{0.386f};
        float lapCompletionThreshold{0.1f};
    } config{};

    ControllerConfig racingConfig{};
    LookaheadIndices lookaheadIndices{};

    double totalTime{0.0};
    double deltaTime{0.0};
    double totalDistance{0.0};
    int lapCount{0};
    fsai::sim::MissionDefinition mission_{};
    fsai::sim::MissionRuntimeState missionState_{};
    bool insideLastCheckpoint_{false};
    struct StraightLineTracker {
        bool valid{false};
        Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
        Eigen::Vector2d direction{Eigen::Vector2d::UnitX()};
        double length{0.0};
    } straightTracker_{};
    void configureMissionRuntime();
    void updateStraightLineProgress();
    void handleMissionCompletion();
    bool crossesCurrentGate(const Vector2& previous, const Vector2& current) const;
};

