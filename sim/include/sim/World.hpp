#pragma once

#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include "types.h"
#include "VehicleState.hpp"
#include "sim/vehicle/VehicleDynamics.hpp"
#include "Telemetry.hpp"
#include "controller.prot.h"
#include "Transform.h"
#include "Vector.h"
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "IWorldView.hpp"
#include "MissionDefinition.hpp"
#include "MissionRuntimeState.hpp"
#include "TrackState.hpp"
#include "TrackBuilder.hpp"

struct WorldConfig {
    fsai::sim::MissionDefinition mission;
    PathConfig pathConfig{};

    WorldConfig() = default;
    explicit WorldConfig(fsai::sim::MissionDefinition missionDefinition)
        : mission(std::move(missionDefinition)) {}
};

class World : public fsai::world::IWorldView {
public:
    World() = default;

    // Initialize the world with vehicle parameters and generate track.
    void init(const VehicleDynamics& vehicleDynamics, const WorldConfig& worldConfig);

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


    const VehicleState& vehicleState() const { return vehicleDynamics().state(); }
    const Transform& vehicleTransform() const { return vehicleDynamics().transform(); }
    const std::vector<Vector3>& checkpointPositionsWorld() const {
        return checkpointPositions;
    }

    const std::vector<Cone>& getStartCones() const { return startCones; }
    const std::vector<Cone>& getLeftCones() const { return leftCones; }
    const std::vector<Cone>& getRightCones() const { return rightCones; }
    const std::vector<Vector3>& getStartConePositions() const { return startConePositions_; }
    const std::vector<Vector3>& getLeftConePositions() const { return leftConePositions_; }
    const std::vector<Vector3>& getRightConePositions() const { return rightConePositions_; }
    const LookaheadIndices& lookahead() const { return lookaheadIndices; }
    const WheelsInfo& wheelsInfo() const { return vehicleDynamics().wheels_info(); }
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
    const DynamicBicycle& model() const { return vehicleDynamics().model(); }

    const fsai::sim::MissionDefinition& mission() const { return mission_; }

    // IWorldView overrides
    const VehicleState& vehicle_state() const override { return vehicleDynamics().state(); }
    const Transform& vehicle_transform() const override { return vehicleDynamics().transform(); }
    const WheelsInfo& wheels_info() const override { return vehicleDynamics().wheels_info(); }
    const std::vector<Vector3>& checkpoint_positions() const override { return checkpointPositions; }
    const std::vector<Vector3>& start_cones() const override { return startConePositions_; }
    const std::vector<Vector3>& left_cones() const override { return leftConePositions_; }
    const std::vector<Vector3>& right_cones() const override { return rightConePositions_; }
    const LookaheadIndices& lookahead_indices() const override { return lookaheadIndices; }
    const fsai::sim::MissionRuntimeState& mission_runtime() const override { return missionState_; }
    double lap_time_seconds() const override { return totalTime; }
    double total_distance_meters() const override { return totalDistance; }
    double time_step_seconds() const override { return deltaTime; }
    int lap_count() const override { return lapCount; }
    const std::vector<std::pair<Vector2, Vector2>>& best_path_edges() const override { return bestPathEdges; }
    const std::vector<FsaiConeDet>& ground_truth_detections() const override { return coneDetections; }
    bool vehicle_reset_pending() const override { return vehicleResetPending_; }
    void acknowledge_vehicle_reset(const Transform& appliedTransform) override {
        acknowledgeVehicleReset(appliedTransform);
    }

    void setVehicleDynamics(const VehicleDynamics& vehicleDynamics);

    struct VehicleSpawnState {
        VehicleState state{Eigen::Vector3d::Zero(), 0.0,
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero()};
        Transform transform{};
    };

    const VehicleSpawnState& vehicleSpawnState() const { return spawnState_; }
    bool vehicleResetPending() const { return vehicleResetPending_; }
    void acknowledgeVehicleReset(const Transform& appliedTransform);

private:
    friend class WorldTestHelper;
    void moveNextCheckpointToLast();
    void reset();
    void configureTrackState(const TrackBuildResult& trackState);
    void initializeVehiclePose();

    const VehicleDynamics* vehicleDynamics_{nullptr};
    VehicleSpawnState spawnState_{};
    bool vehicleResetPending_{false};
    Vector2 prevCarPos_{0.0f, 0.0f};

    std::vector<Vector3> checkpointPositions{};
    std::vector<Cone> startCones{};
    std::vector<Cone> leftCones{};
    std::vector<Cone> rightCones{};
    std::vector<Vector3> startConePositions_{};
    std::vector<Vector3> leftConePositions_{};
    std::vector<Vector3> rightConePositions_{};
    std::vector<CollisionSegment> gateSegments_{};
    std::vector<CollisionSegment> boundarySegments_{};
    Vector3 lastCheckpoint{0.0f, 0.0f, 0.0f};

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
    TrackBuilder trackBuilder_{};
    PathConfig pathConfig_{};
    std::optional<TrackBuildResult> trackState_{};
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
    const VehicleDynamics& vehicleDynamics() const;
};

