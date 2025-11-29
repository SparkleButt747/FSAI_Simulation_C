#pragma once

#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <unordered_map>
#include "VehicleState.hpp"
#include "common/telemetry/Telemetry.hpp"
#include "controller.prot.h"
#include "Transform.h"
#include "Vector.h"
#include "IWorldView.hpp"
#include "MissionDefinition.hpp"
#include "MissionRuntimeState.hpp"
#include "sim/WorldRuntime.hpp"
#include "WorldConfig.hpp"
#include "architecture/WorldDebug.hpp"
#include "TrackBuilder.hpp"
#include "CollisionService.hpp"
#include "ResetPolicy.hpp"
#include "sim/architecture/IVehicleDynamics.hpp"
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "sim/mission/MissionDefinition.hpp"
#include "types.h"
#include "sim/world/TrackTypes.hpp"

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;

struct WorldVehicleSpawn {
    VehicleState state{Eigen::Vector3d::Zero(), 0.0,
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero()};
    Transform transform{};
};
    
using WorldVehicleResetHandler = std::function<void(const WorldVehicleSpawn&)>;

struct WorldVehicleContext {
    fsai::vehicle::IVehicleDynamics* dynamics{nullptr};
    WorldVehicleResetHandler reset_handler{};
};

class World : public fsai::world::IWorldView {
public:
    World();

    // Initialize the world with vehicle parameters and generate track.
    void init(const WorldVehicleContext& vehicleContext, const WorldConfig& worldConfig);

    // Update simulation by dt seconds.
    void update(double dt);

    // Output telemetry information.
    void telemetry() const;

    // Public control parameters for manual control or algorithms.
    float steeringAngle{0.0f};
    float throttleInput{0.0f};
    float brakeInput{0.0f};
    int useController{1};
    int regenTrack{1};
    std::vector<std::pair<Vector2, Vector2>> bestPathEdges {};
    std::vector<std::pair<Vector2, Vector2>> triangulationEdges{}; // Keep an object for the world so that rendering is accurate.

    
    const VehicleState& vehicleState() const { return vehicleDynamics().state(); }
    const Transform& vehicleTransform() const { return vehicleDynamics().transform(); }
    const std::vector<Vector3>& checkpointPositionsWorld() const;

    const std::vector<Cone>& getStartCones() const;
    const std::vector<Cone>& getLeftCones() const;
    const std::vector<Cone>& getRightCones() const;
    
    const std::vector<Vector3>& getStartConePositions() const;
    const std::vector<Vector3>& getLeftConePositions()  const;
    const std::vector<Vector3>& getRightConePositions() const;

    const LookaheadIndices& lookahead() const { return lookaheadIndices; }
    double lapTimeSeconds() const { return runtime_.lap_time_seconds(); }
    double totalDistanceMeters() const { return runtime_.lap_distance_meters(); }
    double timeStepSeconds() const { return runtime_.time_step_seconds(); }
    int completedLaps() const { return runtime_.lap_count(); }
    double missionElapsedSeconds() const { return runtime_.mission_state().mission_time_seconds(); }
    double straightLineProgressMeters() const { return runtime_.mission_state().straight_line_progress_m(); }
    fsai::sim::MissionRunStatus missionRunStatus() const { return runtime_.mission_state().run_status(); }
    const fsai::sim::MissionRuntimeState& missionRuntime() const { return runtime_.mission_state(); }
    fsai::sim::WorldRuntime& runtime_controller() { return runtime_; }
    const fsai::sim::WorldRuntime& runtime_controller() const { return runtime_; }

    bool computeRacingControl(double dt, float& throttle_out, float& steering_out);

    const fsai::sim::MissionDefinition& mission() const { return mission_; }

    // IWorldView overrides
    const VehicleState& vehicle_state() const override { return vehicleDynamics().state(); }
    const Transform& vehicle_transform() const override { return vehicleDynamics().transform(); }
    const WheelsInfo& wheels_info() const override { return vehicleDynamics().wheels_info(); }
    const std::vector<Vector3>& checkpoint_positions() const override {
        enforcePublicGroundTruth("checkpoint positions");
        return checkpointPositions;
    }
    const std::vector<Vector3>& start_cones() const override {
        enforcePublicGroundTruth("start cones");
        return startConePositions_;
    }
    const std::vector<Vector3>& left_cones() const override {
        enforcePublicGroundTruth("left cones");
        return leftConePositions_;
    }
    const std::vector<Vector3>& right_cones() const override {
        enforcePublicGroundTruth("right cones");
        return rightConePositions_;
    }
    const std::vector<Vector3>& orange_cones() const override {
        enforcePublicGroundTruth("orange cones");
        return orangeConePositions_;
    }
    const LookaheadIndices& lookahead_indices() const override { return lookaheadIndices; }
    const fsai::sim::MissionRuntimeState& mission_runtime() const override { return runtime_.mission_state(); }
    double lap_time_seconds() const override { return runtime_.lap_time_seconds(); }
    double total_distance_meters() const override { return runtime_.lap_distance_meters(); }
    double time_step_seconds() const override { return runtime_.time_step_seconds(); }
    int lap_count() const override { return runtime_.lap_count(); }
    std::optional<fsai::sim::WorldRuntime::ResetReason> pending_reset_reason() const override {
        return runtime_.pending_reset_reason();
    }
    const std::vector<std::pair<Vector2, Vector2>>& controller_path_edges() const override {
        return bestPathEdges_;
    }
    const std::vector<std::pair<Vector2, Vector2>>& triangulation_edges() const override {
        return triangulationEdges;
    }
    const std::vector<FsaiConeDet>& debug_detections() const override {
        return coneDetections_;
    }
    bool vehicle_reset_pending() const override { return vehicleResetPending_; }
    void acknowledge_vehicle_reset(const Transform& appliedTransform) override {
        acknowledgeVehicleReset(appliedTransform);
    }

    bool debug_mode() const { return visibilityConfig_.debug_mode; }
    bool public_ground_truth_enabled() const {
        return visibilityConfig_.debug_mode && visibilityConfig_.public_ground_truth;
    }
    bool render_ground_truth_enabled() const {
        return public_ground_truth_enabled() && visibilityConfig_.render_ground_truth;
    }

    void acknowledgeVehicleReset(const Transform& appliedTransform);
    void set_debug_publisher(fsai::world::IWorldDebugPublisher* publisher) {
        debugPublisher_ = publisher;
    }
    void set_debug_detections(std::vector<FsaiConeDet> detections) {
        coneDetections_ = std::move(detections);
    }
    void publish_debug_state() const;

private:
    friend class WorldTestHelper;
    void moveNextCheckpointToLast();
    void reset(const ResetDecision& decision);
    void initializeTriangulation();
    void configureTrackState(const TrackBuildResult& track);
    void initializeVehiclePose();
    TrackBuildResult buildTrackState();
    fsai::sim::TrackData generateRandomTrack() const;
    bool detectCollisions(bool crossedGate);
    void updateStraightLineProgress();
    void handleMissionCompletion();
    bool crossesCurrentGate(const Vector2& previous, const Vector2& current) const;

    fsai::vehicle::IVehicleDynamics* vehicleDynamics_{nullptr};
    WorldVehicleSpawn spawnState_{};
    bool vehicleResetPending_{false};
    WorldVehicleResetHandler vehicleResetHandler_{};
    Vector2 prevCarPos_{0.0f, 0.0f};

    Triangulation triangulation_;
    std::unordered_map<Point, FsaiConeSide> coneToSide_;

    std::vector<Vector3> checkpointPositions{};
    std::vector<Cone> startCones{};
    std::vector<Cone> leftCones{};
    std::vector<Cone> rightCones{};
    std::vector<Cone> orangeCones{};
    std::vector<Vector3> startConePositions_{};
    std::vector<Vector3> leftConePositions_{};
    std::vector<Vector3> rightConePositions_{};
    std::vector<Vector3> orangeConePositions_{};
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

    WorldVisibilityConfig visibilityConfig_{};
    WorldControlConfig controlConfig_{};

    ControllerConfig racingConfig{};
    LookaheadIndices lookaheadIndices{};

    double totalTime{0.0};
    double deltaTime{0.0};
    double totalDistance{0.0};
    int lapCount{0};
    fsai::sim::MissionRuntimeState missionState_{};

    struct StraightLineTracker {
        bool valid{false};
        Eigen::Vector2d origin{0.0, 0.0};
        Eigen::Vector2d direction{1.0, 0.0};
        double length{0.0};
    };

    StraightLineTracker straightTracker_{};

    fsai::sim::MissionDefinition mission_{};
    TrackBuilderConfig trackBuilderConfig_{};
    TrackBuildResult trackState_{};
    bool trackGenerationFailed_{false};
    CollisionService collisionService_{CollisionService::Config{}};
    ResetPolicy resetPolicy_{WorldControlConfig{}};
    fsai::sim::WorldRuntime runtime_{};
    std::vector<std::pair<Vector2, Vector2>> bestPathEdges_{};
    std::vector<FsaiConeDet> coneDetections_{};
    fsai::world::IWorldDebugPublisher* debugPublisher_{nullptr};
    void configureMissionRuntime();
    void bindVehicleDynamics(fsai::vehicle::IVehicleDynamics& vehicleDynamics);
    void publishVehicleSpawn();
    const fsai::vehicle::IVehicleDynamics& vehicleDynamics() const;
    void enforcePublicGroundTruth(const char* accessor) const;
    void publishDebugPacket(const fsai::world::WorldDebugPacket& packet) const;
};
