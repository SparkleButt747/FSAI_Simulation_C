
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <limits>
#include <string>
#include "../include/logging.hpp"
#include "fsai_clock.h"
#include "World.hpp"
#include "centerline.hpp"

using VertexHandle=Triangulation::Vertex_handle;

// WORLD SYSTEM: orchestrates track generation, vehicle binding, collision
// detection, mission timing, and publishes debug/telemetry to downstream
// systems. The flow below highlights how the world owns vehicle dynamics,
// runtime state, and reset policy so fsai_run can remain mostly orchestration.
World::World() {
    runtime_.AddMissionCompleteListener(
        [this](const fsai::sim::MissionRuntimeState&) {
            throttleInput = 0.0f;
            brakeInput = 1.0f;
        });
}

bool World::computeRacingControl(double dt, float& throttle_out, float& steering_out) {
    if (checkpointPositions.empty()) {
        return false;
        std::printf("No checkpoints available for racing control.\n");
    }

    const VehicleState& state = vehicleDynamics().state();
    Vector3 carVelocity{static_cast<float>(state.velocity.x()),
                        static_cast<float>(state.velocity.y()),
                        static_cast<float>(state.velocity.z())};
    const float carSpeed = Vector3_Magnitude(carVelocity);
    const Transform& carTransform = vehicleDynamics().transform();
    Point carFront = getCarFront(state);

    removePassedCones(triangulation_, coneToSide_, carFront, state.yaw);

    if (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration) {
        updateVisibleTrackTriangulation(triangulation_, coneToSide_, carFront, state.yaw, leftCones, rightCones);
    } else {
        updateVisibleTrackTriangulation(triangulation_, coneToSide_, carFront, state.yaw, leftCones, rightCones);
    }

    triangulationEdges.clear();
    for (auto it = triangulation_.finite_edges_begin(); it != triangulation_.finite_edges_end(); ++it) {
        auto seg = triangulation_.segment(*it);
        triangulationEdges.emplace_back(
            Vector2{static_cast<float>(seg.source().x()), static_cast<float>(seg.source().y())},
            Vector2{static_cast<float>(seg.target().x()), static_cast<float>(seg.target().y())}
        );
    }

    auto [nodes, adj] = generateGraph(triangulation_, carFront, coneToSide_);
    auto searchResult =
        (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration)
        ? beamSearch(adj, nodes, carFront,
                     controlConfig_.pathSearchMaxLength,
                     controlConfig_.pathSearchMinLength,
                     controlConfig_.pathSearchBeamWidth,
                     calculateCost_Acceleration)
        : beamSearch(adj, nodes, carFront,
                     controlConfig_.pathSearchMaxLength,
                     controlConfig_.pathSearchMinLength,
                     controlConfig_.pathSearchBeamWidth);
    bestPath_ = searchResult.first;
    bestPathEdges_ = searchResult.second;
    auto beamSearchedCheckpoints = pathNodesToCheckpoints(bestPath_);
    lookaheadIndices = Controller_GetLookaheadIndices(
            static_cast<int>(checkpointPositions.size()), carSpeed, &racingConfig);
    if (!bestPath_.empty()) {
        throttle_out = Controller_GetThrottleInput(
            beamSearchedCheckpoints, static_cast<int>(bestPath_.size()),
            carSpeed, &carTransform, &racingConfig, dt);
        steering_out = Controller_GetSteeringInput(
            beamSearchedCheckpoints, static_cast<int>(bestPath_.size()),
            carSpeed, &carTransform, &racingConfig, dt);
        std::printf("Checkpoints Available: %zu\n", bestPath_.size());
        for (size_t i = 0; i < bestPath_.size(); ++i) {
            const auto& p = beamSearchedCheckpoints[i];
            std::printf("  [%zu] (%f, %f, %f)\n", i, p.x, p.y, p.z);
        }
        for (size_t i = 0; i < nodes.size(); ++i) {
            const auto& n = nodes[i];
            std::printf("Node [%zu]: Midpoint (%f, %f), First Cone (%f, %f), Second Cone (%f, %f)\n",
                        i, n.midpoint.x, n.midpoint.y,
                        n.first.x, n.first.y,
                        n.second.x, n.second.y);
        }
        delete[] beamSearchedCheckpoints;
    } else {
        // Fallback to braking if no path is found
        throttle_out = 0.0f;
        steering_out = 0.0f;
        brakeInput = 1.0f;
        std::printf("No path found by beam search. Applying brakes.\n");
    }


    std::printf("Speed RAW RA: %f, Steer RAW RA: %f ",
            throttle_out,
            steering_out);
    return true;
}

const std::vector<Vector3>& World::checkpointPositionsWorld() const {
    enforcePublicGroundTruth("checkpoint positions");
    return checkpointPositions;
}

const std::vector<Cone>& World::getStartCones() const {
    enforcePublicGroundTruth("start cones");
    return startCones;
}

const std::vector<Cone>& World::getLeftCones() const {
    enforcePublicGroundTruth("left cones");
    return leftCones;
}

const std::vector<Cone>& World::getRightCones() const {
    enforcePublicGroundTruth("right cones");
    return rightCones;
}

const std::vector<Vector3>& World::getStartConePositions() const {
    enforcePublicGroundTruth("start cone positions");
    return startConePositions_;
}

const std::vector<Vector3>& World::getLeftConePositions() const {
    enforcePublicGroundTruth("left cone positions");
    return leftConePositions_;
}

const std::vector<Vector3>& World::getRightConePositions() const {
    enforcePublicGroundTruth("right cone positions");
    return rightConePositions_;
}

void World::bindVehicleDynamics(fsai::vehicle::IVehicleDynamics& vehicleDynamics) {
    vehicleDynamics_ = &vehicleDynamics;
}

void World::acknowledgeVehicleReset(const Transform& appliedTransform) {
    vehicleResetPending_ = false;
    prevCarPos_ = {appliedTransform.position.x, appliedTransform.position.z};
    runtime_.AcknowledgeResetRequest();
}

void World::publishVehicleSpawn() {
    vehicleResetPending_ = true;
    if (vehicleResetHandler_) {
        vehicleResetHandler_(spawnState_);
    }
}

void World::init(const WorldVehicleContext& vehicleContext, const WorldConfig& worldConfig) {
    if (!vehicleContext.dynamics) {
        throw std::invalid_argument("WorldVehicleContext must provide vehicle dynamics");
    }
    bindVehicleDynamics(*vehicleContext.dynamics);
    vehicleResetHandler_ = vehicleContext.reset_handler;
    mission_ = worldConfig.mission;

    visibilityConfig_ = worldConfig.visibility;
    controlConfig_ = worldConfig.control;

    this->config.collisionThreshold = worldConfig.tuning.collisionThreshold;
    this->config.vehicleCollisionRadius = worldConfig.tuning.vehicleCollisionRadius;
    this->config.lapCompletionThreshold = worldConfig.tuning.lapCompletionThreshold;

    collisionService_ = CollisionService(
        CollisionService::Config{config.vehicleCollisionRadius, config.collisionThreshold});
    resetPolicy_ = ResetPolicy(controlConfig_);

    trackBuilderConfig_.vehicleCollisionRadius = config.vehicleCollisionRadius;
    trackBuilderConfig_.pathConfig = PathConfig{};

    // TRACK PIPELINE: generate and load the requested mission layout, then
    // hand the checkpoints, cones, and boundaries into runtime + collision
    // subsystems before seeding the vehicle pose.
    trackState_ = buildTrackState();
    configureTrackState(trackState_);
    configureMissionRuntime();

    useController = controlConfig_.useController ? 1 : 0;
    regenTrack = (controlConfig_.regenerateTrack && mission_.allowRegeneration) ? 1 : 0;

    racingConfig.speedLookAheadSensitivity = controlConfig_.speedLookAheadSensitivity;
    racingConfig.steeringLookAheadSensitivity = controlConfig_.steeringLookAheadSensitivity;
    racingConfig.accelerationFactor = controlConfig_.accelerationFactor;

    initializeVehiclePose();
    publishVehicleSpawn();
}

void World::update(double dt) {
    // WORLD TICK: advance mission runtime, collision detection, and reset
    // policies. All data used by fsai_run (telemetry, render snapshot, reset
    // flags) flows out after this call.
    const auto& dynamics = vehicleDynamics();
    runtime_.BeginStep(dt, dynamics.state());

    if (vehicleResetPending_) {
        return;
    }

    if (trackGenerationFailed_) {
        return;
    }

    if (checkpointPositions.empty()) {
        std::printf("No checkpoints available. Resetting simulation.\n");
        const auto resetReason = fsai::sim::WorldRuntime::ResetReason::kTrackRegeneration;
        runtime_.EmitResetEvent(resetReason);
        reset(resetPolicy_.Decide(resetReason, mission_));
        return;
    }

    const auto& carTransform = dynamics.transform();
    std::printf("Car Rotation: yaw=%.2f",
                carTransform.yaw);
    const Vector2 currentPos{carTransform.position.x, carTransform.position.z};
    const auto collisionResult = collisionService_.Evaluate(prevCarPos_, currentPos, checkpointPositions,
                                                           startCones, leftCones, rightCones, gateSegments_,
                                                           boundarySegments_);

    const Eigen::Vector2d velocity2d(dynamics.state().velocity.x(),
                                     dynamics.state().velocity.y());
    runtime_.AccumulateDistance(velocity2d.norm() * dt);

    if (collisionResult.crossedGate) {
        if (!bestPath_.empty()) {
            const auto& passedNode = bestPath_.front();
            Point p1(passedNode.first.x, passedNode.first.y);
            Point p2(passedNode.second.x, passedNode.second.y);

            VertexHandle v1 = triangulation_.nearest_vertex(p1);
            if (v1->point() == p1) {
                triangulation_.remove(v1);
                coneToSide_.erase(p1);
            }
            VertexHandle v2 = triangulation_.nearest_vertex(p2);
            if (v2->point() == p2) {
                triangulation_.remove(v2);
                coneToSide_.erase(p2);
            }
        }
        triangulationEdges.clear();
        for (auto it = triangulation_.finite_edges_begin(); it != triangulation_.finite_edges_end(); ++it) {
            auto seg = triangulation_.segment(*it);
            triangulationEdges.emplace_back(
                Vector2{static_cast<float>(seg.source().x()), static_cast<float>(seg.source().y())},
                Vector2{static_cast<float>(seg.target().x()), static_cast<float>(seg.target().y())}
            );
        }
        if (auto lap_event = runtime_.RegisterGateCrossing()) {
            if (lap_event->lap_index > 0) {
                std::printf("Lap Completed. Time: %.2f s, Distance: %.2f, Lap: %d\n",
                            lap_event->lap_time_s, lap_event->lap_distance_m,
                            lap_event->lap_index);
            }
        }
    }

    runtime_.UpdateStraightLineProgress(carTransform);

    if (collisionResult.crossedGate && !checkpointPositions.empty()) {
        moveNextCheckpointToLast();
    }

    if (collisionResult.resetReason.has_value()) {
        runtime_.EmitResetEvent(*collisionResult.resetReason);
        reset(resetPolicy_.Decide(*collisionResult.resetReason, mission_));
        return;
    }

    prevCarPos_ = currentPos;

    runtime_.HandleMissionCompletion();
    telemetry();
}

void World::telemetry() const {
    Telemetry_Update(vehicleDynamics().state(), vehicleDynamics().transform(),
                     fsai_clock_now(), runtime_.lap_time_seconds(),
                     runtime_.lap_distance_meters(), runtime_.lap_count(),
                     runtime_.mission_state());
}

void World::configureTrackState(const TrackBuildResult& track) {
    checkpointPositions = track.checkpointPositions;
    startCones = track.startCones;
    leftCones = track.leftCones;
    rightCones = track.rightCones;
    orangeCones = track.orangeCones;
    startConePositions_ = track.startConePositions;
    leftConePositions_ = track.leftConePositions;
    rightConePositions_ = track.rightConePositions;
    orangeConePositions_ = track.orangeConePositions;
    gateSegments_ = track.gateSegments;
    boundarySegments_ = track.boundarySegments;

    if (!checkpointPositions.empty())
        lastCheckpoint = checkpointPositions.back();
    else
        lastCheckpoint = {0.0f, 0.0f, 0.0f};

}

TrackBuildResult World::buildTrackState() {
    TrackBuilder builder(trackBuilderConfig_);
    TrackBuildResult built = builder.Build(mission_);
    mission_.track = built.track;
    return built;
}

void World::configureMissionRuntime() {
    fsai::sim::WorldRuntime::Config runtime_config{};
    runtime_config.lap_completion_threshold = config.lapCompletionThreshold;
    runtime_.Configure(mission_, runtime_config);
    runtime_.UpdateTrackContext(checkpointPositions);
}

void World::initializeVehiclePose() {
    spawnState_ = {};
    if (checkpointPositions.empty()) {
        spawnState_.state = VehicleState(Eigen::Vector3d::Zero(), 0.0,
                                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero());
        spawnState_.transform.position.x = 0.0f;
        spawnState_.transform.position.y = 0.5f;
        spawnState_.transform.position.z = 0.0f;
        spawnState_.transform.yaw = 0.0f;
    }
    else {
        const Vector3& start = checkpointPositions.front();
        const std::size_t lookaheadIndex = checkpointPositions.size() > 1
                                               ? std::min<std::size_t>(2, checkpointPositions.size() - 1)
                                               : 0;
        const Vector3& next = checkpointPositions[lookaheadIndex];
        Vector2 startVector{next.x - start.x, next.z - start.z};

        float startYaw = 0.0f;
        if (checkpointPositions.size() > 1) {
            Vector2 zeroVector{0.0f, 0.0f};
            startYaw = Vector2_SignedAngle(zeroVector, startVector) *
                       (std::numbers::pi_v<float> / 180.0f);
        }

        spawnState_.state = VehicleState(
            Eigen::Vector3d(static_cast<double>(start.x), static_cast<double>(start.z), 0.0),
            startYaw, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        spawnState_.transform.position.x = static_cast<float>(spawnState_.state.position.x());
        spawnState_.transform.position.y = 0.5f;
        spawnState_.transform.position.z = static_cast<float>(spawnState_.state.position.y());
        spawnState_.transform.yaw = static_cast<float>(spawnState_.state.yaw);
    }

    prevCarPos_ = {spawnState_.transform.position.x, spawnState_.transform.position.z};
    runtime_.NotifySpawnApplied(spawnState_.transform);
}

const fsai::vehicle::IVehicleDynamics& World::vehicleDynamics() const {
    if (!vehicleDynamics_) {
        throw std::runtime_error("VehicleDynamics not set for World");
    }
    return *vehicleDynamics_;
}

void World::enforcePublicGroundTruth(const char* accessor) const {
    if (!public_ground_truth_enabled()) {
        throw std::runtime_error(std::string("Access to ") + accessor +
                                 " is disabled when public ground truth is off.");
    }
}

void World::moveNextCheckpointToLast() {
    if (!checkpointPositions.empty()) {
        std::rotate(checkpointPositions.begin(), checkpointPositions.begin() + 1, checkpointPositions.end());
    }
    if (!startCones.empty()) {
        std::rotate(startCones.begin(), startCones.begin() + 1, startCones.end());
    }
    if (!leftCones.empty()) {
        std::rotate(leftCones.begin(), leftCones.begin() + 1, leftCones.end());
    }
    if (!rightCones.empty()) {
        std::rotate(rightCones.begin(), rightCones.begin() + 1, rightCones.end());
    }
    if (!orangeCones.empty()) {
        std::rotate(orangeCones.begin(), orangeCones.begin() + 1, orangeCones.end());
    }
    if (!gateSegments_.empty()) {
        std::rotate(gateSegments_.begin(), gateSegments_.begin() + 1, gateSegments_.end());
    }
    if (!startConePositions_.empty()) {
        std::rotate(startConePositions_.begin(), startConePositions_.begin() + 1, startConePositions_.end());
    }
    if (!leftConePositions_.empty()) {
        std::rotate(leftConePositions_.begin(), leftConePositions_.begin() + 1, leftConePositions_.end());
    }
    if (!rightConePositions_.empty()) {
        std::rotate(rightConePositions_.begin(), rightConePositions_.begin() + 1, rightConePositions_.end());
    }
    if (!orangeConePositions_.empty()) {
        std::rotate(orangeConePositions_.begin(), orangeConePositions_.begin() + 1, orangeConePositions_.end());
    }
}

void World::reset(const ResetDecision& decision) {
    trackGenerationFailed_ = false;
    if (decision.regenerateTrack) {
        std::printf("Regenerating track due to reset.\n");
        if (mission_.trackSource == fsai::sim::TrackSource::kRandom) {
            mission_.track = {};
        }
        try {
            trackState_ = buildTrackState();
        } catch (const std::exception& e) {
            fsai::sim::log::LogWarning(
                std::string("Track regeneration failed: ") + e.what());
            trackGenerationFailed_ = true;
            return;
        }
    } else {
        std::printf("Resetting simulation without regenerating track.\n");
    }

    configureTrackState(trackState_);
    configureMissionRuntime();

    initializeVehiclePose();
    coneDetections_.clear();
    bestPath_.clear();
    bestPathEdges_.clear();
    triangulation_.clear();
    coneToSide_.clear();
    publishVehicleSpawn();
}

void World::publish_debug_state() const {
    if (!debug_mode() || debugPublisher_ == nullptr) {
        return;
    }

    fsai::world::WorldDebugPacket packet{};
    packet.start_cones = startConePositions_;
    packet.left_cones = leftConePositions_;
    packet.right_cones = rightConePositions_;
    packet.orange_cones = orangeConePositions_;
    packet.checkpoints = checkpointPositions;
    packet.controller_path_edges = bestPathEdges_;
    packet.detections = coneDetections_;

    publishDebugPacket(packet);
}

void World::publishDebugPacket(const fsai::world::WorldDebugPacket& packet) const {
    if (debugPublisher_ != nullptr) {
        debugPublisher_->publish_debug_packet(packet);
    }
}
