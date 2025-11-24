#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include "types.h"
#include "VehicleState.hpp"
#include "sim/cone_constants.hpp"
#include "sim/vehicle/VehicleDynamics.hpp"
#include "Telemetry.hpp"
#include "controller.prot.h"
#include "Transform.h"
#include "Vector.h"
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "sim/architecture/IWorldView.hpp"
#include "sim/architecture/WorldDebugPacket.hpp"
#include "sim/mission/MissionDefinition.hpp"
#include "sim/MissionRuntimeState.hpp"
#include "world/CollisionService.hpp"
#include "world/ResetPolicy.hpp"

enum class ConeType {
    Start,
    Left,
    Right,
};

struct WorldConfig {
    fsai::sim::MissionDefinition mission;
    struct DebugSettings {
        bool debug_mode{true};
        bool public_ground_truth{true};
        bool render_cones{true};
        bool render_checkpoints{true};
        bool render_paths{true};
    } debug{};
    struct CollisionConfig {
        float collisionThreshold{1.75f};
        float vehicleCollisionRadius{0.5f - fsai::sim::kSmallConeRadiusMeters};
        float lapCompletionThreshold{0.2f};
    } collision{};
    ControllerConfig controller_defaults{0.5f, 0.0f, 0.0019f};
};

struct VehicleSpawnState {
    VehicleState state{Eigen::Vector3d::Zero(), 0.0,
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero()};
    Transform transform{};
};

struct WorldConfig {
    fsai::sim::MissionDefinition mission;
    CollisionService::Config collision;
    ResetPolicy::Config resetPolicy;
    float vehicleCollisionRadius{0.386f};
    float gateCollisionThreshold{1.75f};
};

struct ResetEvent {
    enum class Reason {
        kInvalidTrack,
        kConeCollision,
        kBoundaryCollision,
    };

    Reason reason{Reason::kInvalidTrack};
    bool regenerateTrack{false};
struct WorldVehicleContext {
    fsai::vehicle::IVehicleDynamics* dynamics{nullptr};
    const DynamicBicycle* dynamics_model{nullptr};
    std::function<void(const VehicleSpawnState&)> reset_vehicle{};
};

class World : public fsai::world::IWorldView {
public:
    World() = default;

    // Initialize the world with vehicle parameters and generate track.
    void init(const WorldVehicleContext& vehicleContext, const WorldConfig& worldConfig);
    void setVehicleContext(const WorldVehicleContext& vehicleContext);

    // Update simulation by dt seconds.
    void update(double dt, const fsai::types::ControlCmd& command);

    // Detect collisions with checkpoints and cones. Returns a reset reason on failure.
    std::optional<ResetEvent::Reason> detectCollisions(bool crossedGate);

    std::optional<ResetEvent> consumeResetEvent() {
        auto out = pendingResetEvent_;
        pendingResetEvent_.reset();
        return out;
    }

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
    const std::vector<Vector3>& checkpointPositionsWorld() const { return debugAccess(checkpointPositions); }

    const std::vector<Cone>& getStartCones() const { return debugAccess(startCones); }
    const std::vector<Cone>& getLeftCones() const { return debugAccess(leftCones); }
    const std::vector<Cone>& getRightCones() const { return debugAccess(rightCones); }
    const std::vector<Vector3>& getStartConePositions() const { return debugAccess(startConePositions_); }
    const std::vector<Vector3>& getLeftConePositions() const { return debugAccess(leftConePositions_); }
    const std::vector<Vector3>& getRightConePositions() const { return debugAccess(rightConePositions_); }
    const LookaheadIndices& lookahead() const { return lookaheadIndices; }
    double lapTimeSeconds() const { return totalTime; }
    double totalDistanceMeters() const { return totalDistance; }
    double timeStepSeconds() const { return deltaTime; }
    int completedLaps() const { return lapCount; }
    double missionElapsedSeconds() const { return missionState_.mission_time_seconds(); }
    double straightLineProgressMeters() const { return missionState_.straight_line_progress_m(); }
    fsai::sim::MissionRunStatus missionRunStatus() const { return missionState_.run_status(); }
    const fsai::sim::MissionRuntimeState& missionRuntime() const { return missionState_; }

    bool computeRacingControl(double dt, float& throttle_out, float& steering_out);
    const DynamicBicycle& model() const;

    const fsai::sim::MissionDefinition& mission() const { return mission_; }

    bool debug_mode() const { return debugConfig_.debug_mode; }
    bool public_ground_truth() const { return debugConfig_.public_ground_truth; }
    bool render_cones() const { return debugConfig_.render_cones; }
    bool render_checkpoints() const { return debugConfig_.render_checkpoints; }
    bool render_paths() const { return debugConfig_.render_paths; }

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
    const std::vector<FsaiConeDet>& ground_truth_detections() const override;
    bool vehicle_reset_pending() const override { return vehicleResetPending_; }
    void acknowledge_vehicle_reset(const Transform& appliedTransform) override {
        acknowledgeVehicleReset(appliedTransform);
    }

    void set_debug_publisher(fsai::world::IWorldDebugPublisher* publisher) { debugPublisher_ = publisher; }
    void update_debug_detections(const std::vector<FsaiConeDet>& detections);

    void setVehicleDynamics(const VehicleDynamics& vehicleDynamics);

    struct VehicleSpawnState {
        VehicleState state{Eigen::Vector3d::Zero(), 0.0,
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero()};
        Transform transform{};
    };

    const VehicleSpawnState& vehicleSpawnState() const { return spawnState_; }
    void acknowledgeVehicleReset(const Transform& appliedTransform);

private:
    friend class WorldTestHelper;
    void moveNextCheckpointToLast();
    void reset(bool regenerateTrack);
    void emitResetEvent(ResetEvent::Reason reason);
    void configureTrackState(const fsai::sim::TrackData& track);
    void rebuildCollisionService();
    void reset();
    void configureTrackState(const TrackBuildResult& trackState);
    void initializeVehiclePose();

    fsai::vehicle::IVehicleDynamics* vehicleDynamics_{nullptr};
    const DynamicBicycle* dynamicsModel_{nullptr};
    std::function<void(const VehicleSpawnState&)> resetVehicle_{};
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

    struct Config {
        float collisionThreshold{2.5f};
        float vehicleCollisionRadius{0.386f};
        float lapCompletionThreshold{0.2f};
    } config{};

    ControllerConfig racingConfig{};
    LookaheadIndices lookaheadIndices{};

    fsai::world::IWorldDebugPublisher* debugPublisher_{nullptr};
    fsai::world::WorldDebugPacket debugPacket_{};

    double totalTime{0.0};
    double deltaTime{0.0};
    int lapCount{0};
    TrackBuilder trackBuilder_{};
    PathConfig pathConfig_{};
    std::optional<TrackBuildResult> trackState_{};
    fsai::sim::MissionDefinition mission_{};
    fsai::sim::MissionRuntimeState missionState_{};
    bool insideLastCheckpoint_{false};
    std::optional<ResetEvent> pendingResetEvent_{};
    struct StraightLineTracker {
        bool valid{false};
        Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
        Eigen::Vector2d direction{Eigen::Vector2d::UnitX()};
        double length{0.0};
    } straightTracker_{};
    fsai::sim::WorldRuntime runtime_{missionState_};
    std::function<void()> onResetRequested_{};
    std::function<void()> onMissionComplete_{};
    void configureMissionRuntime();
    bool crossesCurrentGate(const Vector2& previous, const Vector2& current) const;
    const VehicleDynamics& vehicleDynamics() const;
    ResetPolicy resetPolicy_{};
    std::unique_ptr<CollisionService> collisionService_{};
    template <typename T>
    const T& debugAccess(const T& value) const {
        if (!debugConfig_.debug_mode) {
            throw std::runtime_error("Debug access requested when debug_mode is disabled");
        }
        return value;
    }

    WorldConfig::DebugSettings debugConfig_{};
};

