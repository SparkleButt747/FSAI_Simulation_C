
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

namespace {

using fsai::sim::kLargeConeMassKg;
using fsai::sim::kLargeConeRadiusMeters;
using fsai::sim::kSmallConeMassKg;
using fsai::sim::kSmallConeRadiusMeters;

struct Vec2d {
    double x{0.0};
    double y{0.0};
};

constexpr double kEpsilon = 1e-6;

double cross(const Vec2d& a, const Vec2d& b) {
    return a.x * b.y - a.y * b.x;
}

double orientation(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
    return cross({b.x - a.x, b.y - a.y}, {c.x - a.x, c.y - a.y});
}

bool onSegment(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
    const double minX = std::min(a.x, b.x) - kEpsilon;
    const double maxX = std::max(a.x, b.x) + kEpsilon;
    const double minY = std::min(a.y, b.y) - kEpsilon;
    const double maxY = std::max(a.y, b.y) + kEpsilon;
    return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY;
}

bool segmentsIntersect(const Vec2d& p1, const Vec2d& p2, const Vec2d& q1, const Vec2d& q2) {
    const double o1 = orientation(p1, p2, q1);
    const double o2 = orientation(p1, p2, q2);
    const double o3 = orientation(q1, q2, p1);
    const double o4 = orientation(q1, q2, p2);

    const bool generalCase = ((o1 > kEpsilon && o2 < -kEpsilon) || (o1 < -kEpsilon && o2 > kEpsilon)) &&
                             ((o3 > kEpsilon && o4 < -kEpsilon) || (o3 < -kEpsilon && o4 > kEpsilon));
    if (generalCase) {
        return true;
    }

    if (std::abs(o1) <= kEpsilon && onSegment(p1, p2, q1)) {
        return true;
    }
    if (std::abs(o2) <= kEpsilon && onSegment(p1, p2, q2)) {
        return true;
    }
    if (std::abs(o3) <= kEpsilon && onSegment(q1, q2, p1)) {
        return true;
    }
    if (std::abs(o4) <= kEpsilon && onSegment(q1, q2, p2)) {
        return true;
    }

    return false;
}

Vector3 transformToVector3(const Transform& t) {
    return {t.position.x, t.position.y, t.position.z};
}

Cone makeCone(const Transform& t, ConeType type) {
    Cone cone;
    cone.position = transformToVector3(t);
    cone.type = type;
    switch (type) {
        case ConeType::Start:
            cone.radius = kLargeConeRadiusMeters;
            cone.mass = kLargeConeMassKg;
            break;
        case ConeType::Orange:
        case ConeType::Left:
        case ConeType::Right:
            cone.radius = kSmallConeRadiusMeters;
            cone.mass = kSmallConeMassKg;
            break;
    }
    return cone;
}

CollisionSegment makeSegment(const Vector2& start, const Vector2& end, float radius) {
    CollisionSegment segment{};
    segment.start = start;
    segment.end = end;
    segment.radius = radius;
    const float minX = std::min(start.x, end.x) - radius;
    const float maxX = std::max(start.x, end.x) + radius;
    const float minY = std::min(start.y, end.y) - radius;
    const float maxY = std::max(start.y, end.y) + radius;
    segment.boundsMin = Vector2{minX, minY};
    segment.boundsMax = Vector2{maxX, maxY};
    return segment;
}

float distanceSquaredToSegment(const Vector2& point, const CollisionSegment& segment) {
    const float ax = segment.start.x;
    const float ay = segment.start.y;
    const float bx = segment.end.x;
    const float by = segment.end.y;
    const float abx = bx - ax;
    const float aby = by - ay;
    const float lengthSq = abx * abx + aby * aby;
    if (lengthSq <= std::numeric_limits<float>::epsilon()) {
        const float dx = point.x - ax;
        const float dy = point.y - ay;
        return dx * dx + dy * dy;
    }

    const float t = ((point.x - ax) * abx + (point.y - ay) * aby) / lengthSq;
    const float clamped = std::clamp(t, 0.0f, 1.0f);
    const float closestX = ax + clamped * abx;
    const float closestY = ay + clamped * aby;
    const float dx = point.x - closestX;
    const float dy = point.y - closestY;
    return dx * dx + dy * dy;
}

bool pointWithinBounds(const Vector2& point, const CollisionSegment& segment) {
    return point.x >= segment.boundsMin.x && point.x <= segment.boundsMax.x &&
           point.y >= segment.boundsMin.y && point.y <= segment.boundsMax.y;
}
}  // namespace

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

    std::pair<Triangulation, std::vector<std::pair<Vector2, Vector2>>> triangulation_result_pair;
    std::unordered_map<Point, FsaiConeSide> coneToSide_map;

    if (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration) {
        triangulation_result_pair = getVisibleTriangulationEdges(state, leftCones, rightCones, orangeCones);
        coneToSide_map = getVisibleTrackTriangulationFromCones(getCarFront(state), state.yaw, leftCones, rightCones, orangeCones).second;
    } else {
        triangulation_result_pair = getVisibleTriangulationEdges(state, leftCones, rightCones);
        coneToSide_map = getVisibleTrackTriangulationFromCones(getCarFront(state), state.yaw, leftCones, rightCones).second;
    }

    auto triangulation = triangulation_result_pair.first;
    triangulationEdges = triangulation_result_pair.second;
    auto coneToSide = coneToSide_map;
    removePassedCones(triangulation_, coneToSide_, carFront, carYaw);

    auto [nodes, adj] = generateGraph(triangulation, getCarFront(state), coneToSide);
    auto searchResult =
        (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration)
        ? beamSearch(adj, nodes, getCarFront(state),
                     controlConfig_.pathSearchMaxLength,
                     controlConfig_.pathSearchMinLength,
                     controlConfig_.pathSearchBeamWidth,
                     calculateCost_Acceleration)
        : beamSearch(adj, nodes, getCarFront(state),
                     controlConfig_.pathSearchMaxLength,
                     controlConfig_.pathSearchMinLength,
                     controlConfig_.pathSearchBeamWidth);
    auto pathNodes = searchResult.first;
    bestPathEdges_ = searchResult.second;
    auto beamSearchedCheckpoints = pathNodesToCheckpoints(pathNodes);
    lookaheadIndices = Controller_GetLookaheadIndices(
            static_cast<int>(checkpointPositions.size()), carSpeed, &racingConfig);
    if (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration &&
        !pathNodes.empty()) {
        throttle_out = Controller_GetThrottleInput(
            beamSearchedCheckpoints, static_cast<int>(pathNodes.size()),
            carSpeed, &carTransform, &racingConfig, dt);
        steering_out = Controller_GetSteeringInput(beamSearchedCheckpoints, static_cast<int>(pathNodes.size()), carSpeed, &carTransform, &racingConfig, dt);
        std::printf("Checkpoints Available: %zu\n", pathNodes.size());
        for (size_t i = 0; i < pathNodes.size(); ++i) {
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
        throttle_out = Controller_GetThrottleInput(
            checkpointPositions.data(), static_cast<int>(checkpointPositions.size()),
            carSpeed, &carTransform, &racingConfig, dt);
        steering_out = Controller_GetSteeringInput(
            checkpointPositions.data(), static_cast<int>(checkpointPositions.size()),
            carSpeed, &carTransform, &racingConfig, dt);
        std::printf("Checkpoints Available: %zu\n", checkpointPositions.size());
        for (size_t i = 0; i < checkpointPositions.size(); ++i) {
            const auto& p = checkpointPositions[i];
            std::printf("  [%zu] (%f, %f, %f)\n", i, p.x, p.y, p.z);
        }
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

    if (hasSvcuCommand_ && missionState_.run_status() == fsai::sim::MissionRunStatus::kRunning) {
        throttleInput = lastSvcuThrottle_;
        brakeInput = lastSvcuBrake_;
        steeringAngle = lastSvcuSteer_;
    }

    if (missionState_.run_status() == fsai::sim::MissionRunStatus::kBraking) {
        throttleInput = 0.0f;
        brakeInput = 1.0f;
        if (carState.velocity.norm() < 0.1) {
            missionState_.MarkCompleted();
        }
    }

    hasSvcuCommand_ = false;

    handleMissionCompletion();

    const double netAcc = static_cast<double>(throttleInput - brakeInput);
    carInput.acc = netAcc;
    carInput.delta = steeringAngle;

    carModel.updateState(carState, carInput, dt);

    carTransform.position.x = static_cast<float>(carState.position.x());
    carTransform.position.z = static_cast<float>(carState.position.y());
    carTransform.yaw = static_cast<float>(carState.yaw);

    const Vector2 currentPos{carTransform.position.x, carTransform.position.z};
    const auto collisionResult = collisionService_.Evaluate(prevCarPos_, currentPos, checkpointPositions,
                                                           startCones, leftCones, rightCones, gateSegments_,
                                                           boundarySegments_);

    const Eigen::Vector2d velocity2d(dynamics.state().velocity.x(),
                                     dynamics.state().velocity.y());
    runtime_.AccumulateDistance(velocity2d.norm() * dt);

    if (collisionResult.crossedGate) {
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

    updateStraightLineProgress();
    if (!missionState_.mission_complete()) {
        missionState_.Update(dt);
        totalTime += dt;
    }

    runtime_.HandleMissionCompletion();
    telemetry();
}

bool World::detectCollisions(bool crossedGate) {
    if (crossedGate && !checkpointPositions.empty()) {
        moveNextCheckpointToLast();
    }

    float dx = carTransform.position.x - lastCheckpoint.x;
    float dz = carTransform.position.z - lastCheckpoint.z;
    float distToLast = std::sqrt(dx * dx + dz * dz);
    const bool insideNow = distToLast < config.lapCompletionThreshold;
    if (mission_.descriptor.type != fsai::sim::MissionType::kAcceleration && insideNow && !insideLastCheckpoint_ && !missionState_.mission_complete()) {
        missionState_.RegisterLap(totalTime, totalDistance);
        lapCount = static_cast<int>(missionState_.completed_laps());
        if (lapCount > 0) {
            std::printf("Lap Completed. Time: %.2f s, Distance: %.2f, Lap: %d\n",
                       totalTime, totalDistance, lapCount);
        }
        if (!missionState_.mission_complete()) {
            totalTime = 0.0;
            totalDistance = 0.0;
        }
    }
    insideLastCheckpoint_ = insideNow;

    for (const auto& cone : startCones) {
        float cdx = carTransform.position.x - cone.position.x;
        float cdz = carTransform.position.z - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + config.vehicleCollisionRadius;
        if (cdist < combinedRadius) {
            std::printf("Collision with a cone detected.\n");
            reset();
            return false;
        }
    }
    for (const auto& cone : leftCones) {
        float cdx = carTransform.position.x - cone.position.x;
        float cdz = carTransform.position.z - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + config.vehicleCollisionRadius;
        if (cdist < combinedRadius) {
            std::printf("Collision with a cone detected.\n");
            reset();
            return false;
        }
    }
    for (const auto& cone : rightCones) {
        float cdx = carTransform.position.x - cone.position.x;
        float cdz = carTransform.position.z - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + config.vehicleCollisionRadius;
        if (cdist < combinedRadius) {
            std::printf("Collision with a cone detected.\n");
            reset();
            return false;
        }
    }

    const Vector2 carCenter{carTransform.position.x, carTransform.position.z};
    const float collisionRadius = config.vehicleCollisionRadius;
    const float collisionRadiusSq = collisionRadius * collisionRadius;

    auto segmentHit = [&](const CollisionSegment& segment) {
        if (!pointWithinBounds(carCenter, segment)) {
            return false;
        }
        return distanceSquaredToSegment(carCenter, segment) < collisionRadiusSq;
    };

    for (const auto& segment : gateSegments_) {
        if (segmentHit(segment)) {
            break;
        }
    }

    for (const auto& segment : boundarySegments_) {
        if (segmentHit(segment)) {
            std::printf("Collision with a boundary detected.\n");
            reset();
            return false;
        }
    }

    return true;
}

void World::telemetry() const {
    Telemetry_Update(vehicleDynamics().state(), vehicleDynamics().transform(),
                     fsai_clock_now(), runtime_.lap_time_seconds(),
                     runtime_.lap_distance_meters(), runtime_.lap_count(),
                     runtime_.mission_state());
}

// Add this debug code to World::configureTrackState() in World.cpp
// Place it right after loading the cones from track data

void World::configureTrackState(const fsai::sim::TrackData& track) {
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

    bool isSkidpad = (mission_.descriptor.type == fsai::sim::MissionType::kSkidpad);

    if (!isSkidpad) {
        appendBoundarySegments(leftCones);
        appendBoundarySegments(rightCones);
    }

    // Update last checkpoint
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

    const Vector3& start = checkpointPositions.front();
    const Vector3& finish = checkpointPositions.back();
    if (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration && checkpointPositions.size() >= 2) {
        straightTracker_ = {};
        straightTracker_.valid = false;
        mission_.track_length_m = 75.0; // Hardcode acceleration track length to 75m
        straightTracker_.valid = true;
        const Vector3& start = checkpointPositions.front();
        const Eigen::Vector2d start2(static_cast<double>(start.x), static_cast<double>(start.z));
        straightTracker_.origin = start2;
        straightTracker_.direction = Eigen::Vector2d::UnitX(); // Assuming acceleration is along X-axis
        straightTracker_.length = mission_.track_length_m;
        std::printf("ConfigureMissionRuntime: Hardcoded Acceleration Track Length=%.2f\n", mission_.track_length_m);
    } else if (checkpointPositions.size() < 2) {
        std::printf("ConfigureMissionRuntime: too few checkpoints (%zu)\n", checkpointPositions.size());
    } else {
        const Eigen::Vector2d start2(static_cast<double>(start.x), static_cast<double>(start.z));
        const Eigen::Vector2d finish2(static_cast<double>(finish.x), static_cast<double>(finish.z));
        const Eigen::Vector2d delta = finish2 - start2;
        const double length = delta.norm();
        if (length > 1e-3) {
            straightTracker_.valid = true;
            straightTracker_.origin = start2;
            straightTracker_.direction = delta / length;
            straightTracker_.length = length;
        }
    }
    missionState_.Reset(mission_);
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
    } else {
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

void World::updateStraightLineProgress() {
    if (!straightTracker_.valid) {
        return;
    }
    const Eigen::Vector2d position(static_cast<double>(carTransform.position.x),
                                   static_cast<double>(carTransform.position.z));
    const Eigen::Vector2d offset = position - straightTracker_.origin;
    const double projection = offset.dot(straightTracker_.direction);
    const double clamped = std::clamp(projection, 0.0, straightTracker_.length);
    missionState_.SetStraightLineProgress(clamped);
    std::printf("UpdateStraightLineProgress: Progress=%.2f, Track Length=%.2f\n",
                clamped, mission_.track_length_m);
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

World::handleMissionCompletion() {
    if (!missionState_.mission_complete()) {
        return;
    }
    throttleInput = 0.0f;
    brakeInput = 1.0f;
    if (!missionState_.stop_commanded()) {
        missionState_.MarkStopCommanded();
        std::printf("Mission complete. Commanding stop.\n");
    }
}

bool World::crossesCurrentGate(const Vector2& previous, const Vector2& current) const {
    const Vec2d prev{static_cast<double>(previous.x), static_cast<double>(previous.y)};
    const Vec2d curr{static_cast<double>(current.x), static_cast<double>(current.y)};

    if (leftCones.empty() || rightCones.empty()) {
        /*
        No physical gate from cones available. If we have a planned path from beamSearch,
        use its first edge as a virtual gate between consecutive path nodes. Otherwise
        fall back to the old distance-based checkpoint trigger.
        */
        if (!bestPathEdges.empty()) {
            const auto& gate = bestPathEdges.front();
            const Vec2d left{static_cast<double>(gate.first.x),
                             static_cast<double>(gate.first.y)};
            const Vec2d right{static_cast<double>(gate.second.x),
                              static_cast<double>(gate.second.y)};

            // Check if the segment [prev, curr] (car motion) intersects the gate segment [left, right] (gate).
            const double o1 = orientation(prev, curr, left);
            const double o2 = orientation(prev, curr, right);
            const double o3 = orientation(left, right, prev);
            const double o4 = orientation(left, right, curr);

            auto diffSigns = [](double a, double b)
            {
                return (a > kEpsilon && b < -kEpsilon) || (a < -kEpsilon && b > kEpsilon);
            };

            // Proper intersection
            if (diffSigns(o1, o2) && diffSigns(o3, o4))
            {
                return true;
            }

            // Collinear / edge cases: one of the endpoints lies on the other segment
            if (std::abs(o1) <= kEpsilon && onSegment(prev, curr, left))
            {
                return true;
            }
            if (std::abs(o2) <= kEpsilon && onSegment(prev, curr, right))
            {
                return true;
            }
            if (std::abs(o3) <= kEpsilon && onSegment(left, right, prev))
            {
                return true;
            }
            if (std::abs(o4) <= kEpsilon && onSegment(left, right, curr))
            {
                return true;
            }

            return false;
        }

        // Fallback: still allow simple "pass near checkpoint" detection when no path edges are available
        if (checkpointPositions.empty()) {
            return false;
        }
        const Vector3& checkpoint = checkpointPositions.front();
        const Vec2d cp{static_cast<double>(checkpoint.x), static_cast<double>(checkpoint.z)};
        const double prevDist = std::hypot(prev.x - cp.x, prev.y - cp.y);
        const double currDist = std::hypot(curr.x - cp.x, curr.y - cp.y);
        return currDist < static_cast<double>(config.collisionThreshold) &&
               prevDist >= static_cast<double>(config.collisionThreshold);
    }

    const Vec2d left{static_cast<double>(leftCones.front().position.x),
                     static_cast<double>(leftCones.front().position.z)};
    const Vec2d right{static_cast<double>(rightCones.front().position.x),
                      static_cast<double>(rightCones.front().position.z)};

    const double radius = static_cast<double>(config.vehicleCollisionRadius);
    const double minX = std::min(left.x, right.x) - radius;
    const double maxX = std::max(left.x, right.x) + radius;
    const double minY = std::min(left.y, right.y) - radius;
    const double maxY = std::max(left.y, right.y) + radius;

    if ((prev.x < minX && curr.x < minX) || (prev.x > maxX && curr.x > maxX) ||
        (prev.y < minY && curr.y < minY) || (prev.y > maxY && curr.y > maxY)) {
        return false;
    }

    const Vec2d gateVector{right.x - left.x, right.y - left.y};
    const double gateLenSq = gateVector.x * gateVector.x + gateVector.y * gateVector.y;
    if (gateLenSq <= kEpsilon) {
        return false;
    }

    const double gateLength = std::sqrt(gateLenSq);
    const double margin = gateLength > kEpsilon ? radius / gateLength : 0.0;
    const double prevProj = ((prev.x - left.x) * gateVector.x + (prev.y - left.y) * gateVector.y) / gateLenSq;
    const double currProj = ((curr.x - left.x) * gateVector.x + (curr.y - left.y) * gateVector.y) / gateLenSq;
    const double minProj = -margin;
    const double maxProj = 1.0 + margin;
    if ((prevProj < minProj && currProj < minProj) || (prevProj > maxProj && currProj > maxProj)) {
        return false;
    }

    const double oriPrev = orientation(left, right, prev);
    const double oriCurr = orientation(left, right, curr);
    const bool signChange = (oriPrev > kEpsilon && oriCurr < -kEpsilon) ||
                            (oriPrev < -kEpsilon && oriCurr > kEpsilon);
    if (signChange) {
        return true;
    }

    if (segmentsIntersect(prev, curr, left, right)) {
        return true;
    }

    if (std::abs(oriPrev) <= kEpsilon && onSegment(left, right, prev)) {
        return true;
    }
    if (std::abs(oriCurr) <= kEpsilon && onSegment(left, right, curr)) {
        return true;
    }

    return false;
}

fsai::sim::TrackData World::generateRandomTrack() const {
    PathConfig pathConfig;
    int nPoints = pathConfig.resolution;
    PathGenerator pathGen(pathConfig);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(pathConfig, path);
    return fsai::sim::TrackData::FromTrackResult(track);
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
    if (!bestPathEdges.empty()) {
        std::rotate(bestPathEdges.begin(),
                    bestPathEdges.begin() + 1,
                    bestPathEdges.end());
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
    carInput = VehicleInput(0.0, 0.0, 0.0);
    totalTime = 0.0;
    totalDistance = 0.0;
    lapCount = 0;
    insideLastCheckpoint_ = false;

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

    const float initDx = carTransform.position.x - lastCheckpoint.x;
    const float initDz = carTransform.position.z - lastCheckpoint.z;
    const float initDist = std::sqrt(initDx * initDx + initDz * initDz);
    insideLastCheckpoint_ = initDist < config.lapCompletionThreshold;
    coneDetections_.clear();
    bestPathEdges_.clear();
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
