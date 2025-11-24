
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <limits>
#include "fsai_clock.h"
#include "World.hpp"
#include "cone_constants.hpp"
#include "vehicle/VehicleDynamics.hpp"
#include "centerline.hpp"

namespace {

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
    if (!useController || checkpointPositions.empty()) {
        return false;
    }

    const VehicleState& state = vehicleDynamics().state();
    Vector3 carVelocity{static_cast<float>(state.velocity.x()),
                        static_cast<float>(state.velocity.y()),
                        static_cast<float>(state.velocity.z())};
    const float carSpeed = Vector3_Magnitude(carVelocity);
    const Transform& carTransform = vehicleDynamics().transform();

    auto triangulation = getVisibleTriangulationEdges(vehicleState(), leftCones, rightCones).first;
    auto coneToSide = getVisibleTrackTriangulationFromCones(getCarFront(vehicleState()), vehicleState().yaw, leftCones, rightCones).second;
    auto [nodes, adj] = generateGraph(triangulation, getCarFront(vehicleState()), coneToSide);
    auto searchResult = beamSearch(adj, nodes, getCarFront(vehicleState()), 30, 2, 20);
    auto pathNodes = searchResult.first;
    debugPacket_.controller_path = searchResult.second;
    auto checkpoints = pathNodesToCheckpoints(pathNodes);
    lookaheadIndices = Controller_GetLookaheadIndices(
        static_cast<int>(checkpointPositions.size()), carSpeed, &racingConfig);
    throttle_out = Controller_GetThrottleInput(
        checkpointPositions.data(), static_cast<int>(checkpointPositions.size()),
        carSpeed, &carTransform, &racingConfig, dt);
    steering_out = Controller_GetSteeringInput(
        checkpointPositions.data(), static_cast<int>(checkpointPositions.size()),
        carSpeed, &carTransform, &racingConfig, dt);
    if (debugPublisher_) {
        debugPublisher_->publish(debugPacket_);
    }

    return true;
}

void World::setSvcuCommand(float throttle, float brake, float steer) {
    lastSvcuThrottle_ = throttle;
    lastSvcuBrake_ = brake;
    lastSvcuSteer_ = steer;
    hasSvcuCommand_ = true;
}

void World::update_debug_detections(const std::vector<FsaiConeDet>& detections) {
    debugPacket_.detections = detections;
    if (debugPublisher_) {
        debugPublisher_->publish(debugPacket_);
    }
}

void World::setVehicleDynamics(const VehicleDynamics& vehicleDynamics) {
    vehicleDynamics_ = &vehicleDynamics;
const DynamicBicycle& World::model() const {
    return vehicleModel();
}

void World::acknowledgeVehicleReset(const Transform& appliedTransform) {
    vehicleResetPending_ = false;
    prevCarPos_ = {appliedTransform.position.x, appliedTransform.position.z};
}

void World::setVehicleContext(const WorldVehicleContext& vehicleContext) {
    if (!vehicleContext.dynamics) {
        throw std::invalid_argument("WorldVehicleContext.dynamics is required");
    }
    vehicleDynamics_ = vehicleContext.dynamics;
    dynamicsModel_ = vehicleContext.dynamics_model;
    if (!dynamicsModel_) {
        if (auto* concrete = dynamic_cast<VehicleDynamics*>(vehicleDynamics_)) {
            dynamicsModel_ = &concrete->model();
        }
    }
    resetVehicle_ = vehicleContext.reset_vehicle;
}

void World::init(const WorldVehicleContext& vehicleContext, const WorldConfig& worldConfig) {
    setVehicleContext(vehicleContext);
    mission_ = worldConfig.mission;
    debugConfig_ = worldConfig.debug;

    if (mission_.trackSource == fsai::sim::TrackSource::kRandom &&
        mission_.track.checkpoints.empty()) {
        mission_.track = generateRandomTrack();
    }

    if (mission_.track.checkpoints.empty()) {
        throw std::runtime_error("MissionDefinition did not provide any checkpoints");
    }

    this->config.collisionThreshold = worldConfig.runtime.collisionThreshold;
    this->config.vehicleCollisionRadius = worldConfig.runtime.vehicleCollisionRadius;
    this->config.lapCompletionThreshold = worldConfig.runtime.lapCompletionThreshold;
    this->config.collisionThreshold = worldConfig.gateCollisionThreshold;
    this->config.vehicleCollisionRadius = worldConfig.vehicleCollisionRadius;
    this->config.lapCompletionThreshold = worldConfig.collision.lapCompletionThreshold;
    this->resetPolicy_ = ResetPolicy(worldConfig.resetPolicy);
    this->config.collisionThreshold = worldConfig.collision.collisionThreshold;
    this->config.vehicleCollisionRadius = worldConfig.collision.vehicleCollisionRadius;
    this->config.lapCompletionThreshold = worldConfig.collision.lapCompletionThreshold;

    trackState_ = trackBuilder_.Build(mission_, pathConfig_, this->config.vehicleCollisionRadius);
    mission_.track = trackState_->track;

    configureTrackState(*trackState_);
    configureMissionRuntime();

    lapCount = 0;
    deltaTime = 0.0;

    useController = 1;

    racingConfig.speedLookAheadSensitivity = worldConfig.runtime.speedLookAheadSensitivity;
    racingConfig.steeringLookAheadSensitivity = worldConfig.runtime.steeringLookAheadSensitivity;
    racingConfig.accelerationFactor = worldConfig.runtime.accelerationFactor;
    racingConfig.speedLookAheadSensitivity = worldConfig.controller_defaults.speedLookAheadSensitivity;
    racingConfig.steeringLookAheadSensitivity = worldConfig.controller_defaults.steeringLookAheadSensitivity;
    racingConfig.accelerationFactor = worldConfig.controller_defaults.accelerationFactor;

    initializeVehiclePose();
    applyVehicleSpawn();
}

const std::vector<FsaiConeDet>& World::ground_truth_detections() const {
    if (!debugConfig_.public_ground_truth) {
        static const std::vector<FsaiConeDet> kEmptyDetections;
        return kEmptyDetections;
    }
    return coneDetections;
}

void World::update(double dt) {
    const auto& dynamics = vehicleDynamics();
    deltaTime = dt;

    throttleInput = command.throttle;
    brakeInput = command.brake;
    steeringAngle = command.steer_rad;

    if (vehicleResetPending_) {
        return;
    }

    if (checkpointPositions.empty()) {
        std::printf("No checkpoints available. Resetting simulation.\n");
        emitResetEvent(ResetEvent::Reason::kInvalidTrack);
        return;
    }

    handleMissionCompletion();

    const auto& carTransform = dynamics.transform();

    const Vector2 currentPos{carTransform.position.x, carTransform.position.z};
    const bool crossedGate = crossesCurrentGate(prevCarPos_, currentPos);

    runtime_.AdvanceMission(dt, {dynamics.state().velocity.x(), dynamics.state().velocity.y()});

    if (auto resetReason = detectCollisions(crossedGate)) {
        emitResetEvent(*resetReason);
        return;
    }

    prevCarPos_ = currentPos;

    runtime_.UpdateStraightLineProgress(carTransform);
    runtime_.CheckMissionComplete();

    telemetry();
}

std::optional<ResetEvent::Reason> World::detectCollisions(bool crossedGate) {
    const Transform& carTransform = vehicleDynamics().transform();

    if (crossedGate && !checkpointPositions.empty()) {
        moveNextCheckpointToLast();
    }

    if (!collisionService_) {
        return ResetEvent::Reason::kInvalidTrack;
    }

    const auto collisionResult =
        collisionService_->Evaluate(carTransform, lastCheckpoint, insideLastCheckpoint_);

    if (collisionResult.lapCompleted && !missionState_.mission_complete()) {
        missionState_.RegisterLap(totalTime, totalDistance);
    if (auto lap = runtime_.EvaluateLapCrossing(carTransform, lastCheckpoint)) {
        lapCount = static_cast<int>(missionState_.completed_laps());
        std::printf("Lap Completed. Time: %.2f s, Distance: %.2f, Lap: %d\n",
                   lap->time_seconds, lap->distance_meters, lapCount);
    }

    for (const auto& cone : startCones) {
        float cdx = carTransform.position.x - cone.position.x;
        float cdz = carTransform.position.z - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + runtimeConfig_.vehicle_collision_radius;
        if (cdist < combinedRadius) {
            std::printf("Collision with a cone detected.\n");
            runtime_.RequestReset();
            reset();
            return false;
        }
    }
    for (const auto& cone : leftCones) {
        float cdx = carTransform.position.x - cone.position.x;
        float cdz = carTransform.position.z - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + runtimeConfig_.vehicle_collision_radius;
        if (cdist < combinedRadius) {
            std::printf("Collision with a cone detected.\n");
            runtime_.RequestReset();
            reset();
            return false;
        }
    }
    for (const auto& cone : rightCones) {
        float cdx = carTransform.position.x - cone.position.x;
        float cdz = carTransform.position.z - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + runtimeConfig_.vehicle_collision_radius;
        if (cdist < combinedRadius) {
            std::printf("Collision with a cone detected.\n");
            runtime_.RequestReset();
            reset();
            return false;
        }
    }

    const Vector2 carCenter{carTransform.position.x, carTransform.position.z};
    const float collisionRadius = runtimeConfig_.vehicle_collision_radius;
    const float collisionRadiusSq = collisionRadius * collisionRadius;

    insideLastCheckpoint_ = collisionResult.insideLapZone;

    if (collisionResult.coneCollision) {
        std::printf("Collision with a cone detected.\n");
        return ResetEvent::Reason::kConeCollision;
    }
    if (collisionResult.boundaryCollision) {
        std::printf("Collision with a boundary detected.\n");
        return ResetEvent::Reason::kBoundaryCollision;

    for (const auto& segment : boundarySegments_) {
        if (segmentHit(segment)) {
            std::printf("Collision with a boundary detected.\n");
            runtime_.RequestReset();
            reset();
            return false;
        }
    }

    return std::nullopt;
}

void World::emitResetEvent(ResetEvent::Reason reason) {
    const bool regenerateTrack = resetPolicy_.ShouldRegenerate(mission_);
    pendingResetEvent_ = ResetEvent{reason, regenerateTrack};
    reset(regenerateTrack);
}

void World::telemetry() const {
    Telemetry_Update(vehicleDynamics().state(), vehicleDynamics().transform(),
                     fsai_clock_now(), runtime_.lap_time_seconds(), runtime_.lap_distance_meters(), lapCount,
                     missionState_);
}

// Add this debug code to World::configureTrackState() in World.cpp
// Place it right after loading the cones from track data

void World::configureTrackState(const fsai::sim::TrackData& track) {
    checkpointPositions.clear();
    startCones.clear();
    leftCones.clear();
    rightCones.clear();
    startConePositions_.clear();
    leftConePositions_.clear();
    rightConePositions_.clear();
    gateSegments_.clear();
    boundarySegments_.clear();

    // Load checkpoints
    for (const auto& cp : track.checkpoints)
        checkpointPositions.push_back(transformToVector3(cp));

    // Load cones
    for (const auto& sc : track.startCones)
        startCones.push_back(makeCone(sc, ConeType::Start));

    for (const auto& lc : track.leftCones)
        leftCones.push_back(makeCone(lc, ConeType::Left));

    for (const auto& rc : track.rightCones)
        rightCones.push_back(makeCone(rc, ConeType::Right));

    debugPacket_.checkpoints = checkpointPositions;
    debugPacket_.start_cones.clear();
    debugPacket_.left_cones.clear();
    debugPacket_.right_cones.clear();
    debugPacket_.detections.clear();

    bool isSkidpad = (mission_.descriptor.type == fsai::sim::MissionType::kSkidpad);

    // =========================================================================
    // DEBUG: Print orange cone positions
    // =========================================================================
    // Add this to World::configureTrackState() in the skidpad section
    if (isSkidpad)
    {
        // Keep existing orange cones (the 4 big ones at top)
        // Now ADD corridor cones (entrance/exit) as orange
        
        // Circle centers based on your data:
        // Left circle: center around X=-9.25, Z=0
        // Right circle: center around X=9.25, Z=0
        const double leftCircleCenterX = -9.25;
        const double rightCircleCenterX = 9.25;
        const double circleCenterZ = 0.0;
        const double circleRadius = 9.0; // Approximate radius from data
        const double corridorMargin = 2.5; // Distance outside circle to consider "corridor"
        
        auto moveCorridorConesToOrange = [&](std::vector<Cone>& list) {
            std::vector<Cone> keep;
            for (auto& c : list) {
                // Calculate distance from both circle centers
                double distLeft = std::hypot(c.position.x - leftCircleCenterX, 
                                            c.position.z - circleCenterZ);
                double distRight = std::hypot(c.position.x - rightCircleCenterX, 
                                            c.position.z - circleCenterZ);
                
                // If cone is NOT part of either circle (too far from both centers)
                // then it's a corridor cone
                bool isCorridorCone = (distLeft > circleRadius + corridorMargin) && 
                                    (distRight > circleRadius + corridorMargin);
                
                if (isCorridorCone) {
                    c.type = ConeType::Start;  // Make it ORANGE
                    startCones.push_back(c);
                } else {
                    keep.push_back(c);
                }
            }
            list = keep;
        };
        
        moveCorridorConesToOrange(leftCones);
        moveCorridorConesToOrange(rightCones);
        
        // Sort orange cones by Z so entrance (bottom) comes first
        std::sort(startCones.begin(), startCones.end(), 
                [](const Cone& a, const Cone& b) { 
                    return a.position.z < b.position.z; 
                });
        
        std::printf("After processing: Total orange cones = %zu\n", startCones.size());
        
        // Mirror the entire track along Z axis (flip upside down)
        // So entrance moves from top to bottom
        auto flipZ = [](std::vector<Cone>& cones) {
            for (auto& c : cones) {
                c.position.z = -c.position.z;
            }
        };
        
        flipZ(startCones);
        flipZ(leftCones);
        flipZ(rightCones);
        
        // Also flip checkpoints
        for (auto& cp : checkpointPositions) {
            cp.z = -cp.z;
        }
        
        std::printf("Track flipped: entrance now at bottom (negative Z)\n");
        
        // Swap blue and yellow cones
        std::swap(leftCones, rightCones);

        std::printf("Blue and yellow cones swapped\n");
    }
    // =========================================================================

    auto rebuildConePositions = [&]() {
        startConePositions_.clear();
        leftConePositions_.clear();
        rightConePositions_.clear();
        startConePositions_.reserve(startCones.size());
        leftConePositions_.reserve(leftCones.size());
        rightConePositions_.reserve(rightCones.size());
        for (const auto& cone : startCones) {
            startConePositions_.push_back(cone.position);
        }
        for (const auto& cone : leftCones) {
            leftConePositions_.push_back(cone.position);
        }
        for (const auto& cone : rightCones) {
            rightConePositions_.push_back(cone.position);
        }
    };

    rebuildConePositions();

    debugPacket_.start_cones = startConePositions_;
    debugPacket_.left_cones = leftConePositions_;
    debugPacket_.right_cones = rightConePositions_;
    if (debugPublisher_) {
        debugPublisher_->publish(debugPacket_);
    }

    // Rest of your original code continues here...
    if (!leftCones.empty() && !rightCones.empty()) {
        const std::size_t gateCount = std::min(leftCones.size(), rightCones.size());
        gateSegments_.reserve(gateCount);
        for (std::size_t i = 0; i < gateCount; ++i) {
            Vector2 left{leftCones[i].position.x, leftCones[i].position.z};
            Vector2 right{rightCones[i].position.x, rightCones[i].position.z};
            gateSegments_.push_back(makeSegment(left, right, config.vehicleCollisionRadius));
        }
    }

    auto appendBoundarySegments = [&](const std::vector<Cone>& cones) {
        if (cones.size() < 2) return;

        for (std::size_t i = 0; i + 1 < cones.size(); ++i) {
            Vector2 s{cones[i].position.x, cones[i].position.z};
            Vector2 e{cones[i+1].position.x, cones[i+1].position.z};
            boundarySegments_.push_back(makeSegment(s, e, config.vehicleCollisionRadius));
        }

        // close loop
        Vector2 s{cones.back().position.x, cones.back().position.z};
        Vector2 e{cones.front().position.x, cones.front().position.z};
        boundarySegments_.push_back(makeSegment(s, e, config.vehicleCollisionRadius));
    };

    if (!isSkidpad) {
        appendBoundarySegments(leftCones);
        appendBoundarySegments(rightCones);
    }

    // Update last checkpoint
    if (!track.checkpoints.empty())
        lastCheckpoint = transformToVector3(track.checkpoints.back());
    else
        lastCheckpoint = {0.0f, 0.0f, 0.0f};

    rebuildCollisionService();
}

void World::rebuildCollisionService() {
    CollisionService::Config collisionConfig{};
    collisionConfig.lapCompletionThreshold = config.lapCompletionThreshold;
    collisionService_ = std::make_unique<CollisionService>(
        collisionConfig, startCones, leftCones, rightCones, gateSegments_, boundarySegments_,
        config.vehicleCollisionRadius);
void World::configureTrackState(const TrackBuildResult& trackState) {
    checkpointPositions = trackState.checkpointPositions;
    startCones = trackState.startCones;
    leftCones = trackState.leftCones;
    rightCones = trackState.rightCones;
    startConePositions_ = trackState.startConePositions;
    leftConePositions_ = trackState.leftConePositions;
    rightConePositions_ = trackState.rightConePositions;
    gateSegments_ = trackState.gateSegments;
    boundarySegments_ = trackState.boundarySegments;
    lastCheckpoint = trackState.lastCheckpoint;
}

void World::configureMissionRuntime() {
    runtime_.Configure(mission_, checkpointPositions, runtimeConfig_);
    runtime_.set_events({
        [this]() {
            if (onResetRequested_) {
                onResetRequested_();
            }
        },
        [this]() {
            throttleInput = 0.0f;
            brakeInput = 1.0f;
            steeringAngle = 0.0f;
            if (onMissionComplete_) {
                onMissionComplete_();
            }
        },
    });
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
}

void World::applyVehicleSpawn() {
    vehicleResetPending_ = true;
    if (resetVehicle_) {
        resetVehicle_(spawnState_);
    } else {
        vehicleDynamics().set_state(spawnState_.state, spawnState_.transform);
    }
    acknowledgeVehicleReset(spawnState_.transform);
}

void World::updateStraightLineProgress() {
    if (!straightTracker_.valid) {
        return;
    }
    const Transform& carTransform = vehicleDynamics().transform();
    const Eigen::Vector2d position(static_cast<double>(carTransform.position.x),
                                   static_cast<double>(carTransform.position.z));
    const Eigen::Vector2d offset = position - straightTracker_.origin;
    const double projection = offset.dot(straightTracker_.direction);
    const double clamped = std::clamp(projection, 0.0, straightTracker_.length);
    missionState_.SetStraightLineProgress(clamped);
}

void World::handleMissionCompletion() {
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
        if (checkpointPositions.empty()) {
            return false;
        }
        const Vector3& checkpoint = checkpointPositions.front();
        const Vec2d cp{static_cast<double>(checkpoint.x), static_cast<double>(checkpoint.z)};
        const double prevDist = std::hypot(prev.x - cp.x, prev.y - cp.y);
        const double currDist = std::hypot(curr.x - cp.x, curr.y - cp.y);
        return currDist < static_cast<double>(runtimeConfig_.collision_threshold) &&
               prevDist >= static_cast<double>(runtimeConfig_.collision_threshold);
    }

    const Vec2d left{static_cast<double>(leftCones.front().position.x),
                     static_cast<double>(leftCones.front().position.z)};
    const Vec2d right{static_cast<double>(rightCones.front().position.x),
                      static_cast<double>(rightCones.front().position.z)};

    const double radius = static_cast<double>(runtimeConfig_.vehicle_collision_radius);
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

const fsai::vehicle::IVehicleDynamics& World::vehicleDynamics() const {
    if (!vehicleDynamics_) {
        throw std::runtime_error("VehicleDynamics not set for World");
    }
    return *vehicleDynamics_;
}

void World::moveNextCheckpointToLast() {
    if (!checkpointPositions.empty()) {
        std::rotate(checkpointPositions.begin(), checkpointPositions.begin() + 1, checkpointPositions.end());
    }
    if (!leftCones.empty()) {
        std::rotate(leftCones.begin(), leftCones.begin() + 1, leftCones.end());
    }
    if (!rightCones.empty()) {
        std::rotate(rightCones.begin(), rightCones.begin() + 1, rightCones.end());
    }
    if (!gateSegments_.empty()) {
        std::rotate(gateSegments_.begin(), gateSegments_.begin() + 1, gateSegments_.end());
    }
    startConePositions_.clear();
    leftConePositions_.clear();
    rightConePositions_.clear();
    for (const auto& cone : startCones) {
        startConePositions_.push_back(cone.position);
    }
    for (const auto& cone : leftCones) {
        leftConePositions_.push_back(cone.position);
    }
    for (const auto& cone : rightCones) {
        rightConePositions_.push_back(cone.position);
    }
    debugPacket_.checkpoints = checkpointPositions;
    debugPacket_.start_cones = startConePositions_;
    debugPacket_.left_cones = leftConePositions_;
    debugPacket_.right_cones = rightConePositions_;
    if (debugPublisher_) {
        debugPublisher_->publish(debugPacket_);
    }
}

void World::reset(bool regenerateTrack) {
    totalTime = 0.0;
    totalDistance = 0.0;
void World::reset() {
    lapCount = 0;

    if (regenerateTrack) {
        std::printf("Regenerating track due to cone collision.\n");
        if (mission_.trackSource == fsai::sim::TrackSource::kRandom) {
            mission_.track = {};
        }
        trackState_ = trackBuilder_.Build(mission_, pathConfig_, config.vehicleCollisionRadius);
        mission_.track = trackState_->track;
    } else {
        std::printf("Resetting simulation without regenerating track.\n");
        if (!trackState_) {
            trackState_ = trackBuilder_.Build(mission_, pathConfig_, config.vehicleCollisionRadius);
            mission_.track = trackState_->track;
        }
    }

    if (trackState_) {
        configureTrackState(*trackState_);
    }

    configureMissionRuntime();

    initializeVehiclePose();
    applyVehicleSpawn();

    const float initDx = spawnState_.transform.position.x - lastCheckpoint.x;
    const float initDz = spawnState_.transform.position.z - lastCheckpoint.z;
    const float initDist = std::sqrt(initDx * initDx + initDz * initDz);
    insideLastCheckpoint_ = initDist < config.lapCompletionThreshold;
    debugPacket_.controller_path.clear();
    debugPacket_.detections.clear();
    if (debugPublisher_) {
        debugPublisher_->publish(debugPacket_);
    }
    vehicleResetPending_ = true;
}
    coneDetections.clear();
}
