
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <limits>
#include <string>
#include "fsai_clock.h"
#include "World.hpp"
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

    auto triangulation =
        getVisibleTriangulationEdges(vehicleState(), leftCones, rightCones).first;
    auto coneToSide = getVisibleTrackTriangulationFromCones(
                          getCarFront(vehicleState()), vehicleState().yaw, leftCones, rightCones)
                          .second;
    auto [nodes, adj] = generateGraph(triangulation, getCarFront(vehicleState()), coneToSide);
    auto searchResult = beamSearch(adj, nodes, getCarFront(vehicleState()), 30, 2, 20);
    auto pathNodes = searchResult.first;
    bestPathEdges = searchResult.second;
    auto checkpoints = pathNodesToCheckpoints(pathNodes);
    lookaheadIndices = Controller_GetLookaheadIndices(
        static_cast<int>(checkpointPositions.size()), carSpeed, &racingConfig);
    throttle_out = Controller_GetThrottleInput(
        checkpointPositions.data(), static_cast<int>(checkpointPositions.size()),
        carSpeed, &carTransform, &racingConfig, dt);
    steering_out = Controller_GetSteeringInput(
        checkpointPositions.data(), static_cast<int>(checkpointPositions.size()),
        carSpeed, &carTransform, &racingConfig, dt);
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

void World::setSvcuCommand(float throttle, float brake, float steer) {
    lastSvcuThrottle_ = throttle;
    lastSvcuBrake_ = brake;
    lastSvcuSteer_ = steer;
    hasSvcuCommand_ = true;
}

void World::setVehicleDynamics(const VehicleDynamics& vehicleDynamics) {
    vehicleDynamics_ = &vehicleDynamics;
}

void World::acknowledgeVehicleReset(const Transform& appliedTransform) {
    vehicleResetPending_ = false;
    prevCarPos_ = {appliedTransform.position.x, appliedTransform.position.z};
}

void World::init(const VehicleDynamics& vehicleDynamics, const WorldConfig& worldConfig) {
    setVehicleDynamics(vehicleDynamics);
    mission_ = worldConfig.mission;

    visibilityConfig_ = worldConfig.visibility;
    controlConfig_ = worldConfig.control;

    this->config.collisionThreshold = worldConfig.tuning.collisionThreshold;
    this->config.vehicleCollisionRadius = worldConfig.tuning.vehicleCollisionRadius;
    this->config.lapCompletionThreshold = worldConfig.tuning.lapCompletionThreshold;

    trackBuilderConfig_.vehicleCollisionRadius = config.vehicleCollisionRadius;
    trackBuilderConfig_.pathConfig = PathConfig{};

    trackState_ = buildTrackState();
    configureTrackState(trackState_);
    configureMissionRuntime();

    totalTime = 0.0;
    totalDistance = 0.0;
    lapCount = 0;
    deltaTime = 0.0;

    useController = controlConfig_.useController ? 1 : 0;
    regenTrack = (controlConfig_.regenerateTrack && mission_.allowRegeneration) ? 1 : 0;

    racingConfig.speedLookAheadSensitivity = controlConfig_.speedLookAheadSensitivity;
    racingConfig.steeringLookAheadSensitivity = controlConfig_.steeringLookAheadSensitivity;
    racingConfig.accelerationFactor = controlConfig_.accelerationFactor;

    initializeVehiclePose();
    vehicleResetPending_ = true;

    hasSvcuCommand_ = false;
    lastSvcuThrottle_ = 0.0f;
    lastSvcuBrake_ = 0.0f;
    lastSvcuSteer_ = 0.0f;
}

void World::update(double dt) {
    const auto& dynamics = vehicleDynamics();
    deltaTime = dt;

    if (vehicleResetPending_) {
        return;
    }

    if (!missionState_.mission_complete()) {
        missionState_.Update(dt);
        totalTime += dt;
    }

    if (checkpointPositions.empty()) {
        std::printf("No checkpoints available. Resetting simulation.\n");
        reset();
        return;
    }

    if (hasSvcuCommand_ && missionState_.run_status() == fsai::sim::MissionRunStatus::kRunning) {
        throttleInput = lastSvcuThrottle_;
        brakeInput = lastSvcuBrake_;
        steeringAngle = lastSvcuSteer_;
    }

    hasSvcuCommand_ = false;

    handleMissionCompletion();

    const auto& carTransform = dynamics.transform();

    const Vector2 currentPos{carTransform.position.x, carTransform.position.z};
    const bool crossedGate = crossesCurrentGate(prevCarPos_, currentPos);

    const Eigen::Vector2d velocity2d(dynamics.state().velocity.x(),
                                     dynamics.state().velocity.y());
    if (!missionState_.mission_complete()) {
        totalDistance += velocity2d.norm() * dt;
    }

    if (!detectCollisions(crossedGate)) {
        return;
    }

    prevCarPos_ = currentPos;

    updateStraightLineProgress();
    handleMissionCompletion();

    telemetry();
}

bool World::detectCollisions(bool crossedGate) {
    const Transform& carTransform = vehicleDynamics().transform();

    if (crossedGate && !checkpointPositions.empty()) {
        moveNextCheckpointToLast();
    }

    float dx = carTransform.position.x - lastCheckpoint.x;
    float dz = carTransform.position.z - lastCheckpoint.z;
    float distToLast = std::sqrt(dx * dx + dz * dz);
    const bool insideNow = distToLast < config.lapCompletionThreshold;
    if (insideNow && !insideLastCheckpoint_ && !missionState_.mission_complete()) {
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
                     fsai_clock_now(), totalTime, totalDistance, lapCount,
                     missionState_);
}

void World::configureTrackState(const TrackBuildResult& track) {
    checkpointPositions = track.checkpointPositions;
    startCones = track.startCones;
    leftCones = track.leftCones;
    rightCones = track.rightCones;
    startConePositions_ = track.startConePositions;
    leftConePositions_ = track.leftConePositions;
    rightConePositions_ = track.rightConePositions;
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
    missionState_.Reset(mission_);
    straightTracker_ = {};
    straightTracker_.valid = false;

    if (mission_.descriptor.type == fsai::sim::MissionType::kAcceleration && checkpointPositions.size() >= 2) {
        const Vector3& start = checkpointPositions.front();
        const Vector3& finish = checkpointPositions.back();
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

const VehicleDynamics& World::vehicleDynamics() const {
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
    if (!leftCones.empty()) {
        std::rotate(leftCones.begin(), leftCones.begin() + 1, leftCones.end());
    }
    if (!rightCones.empty()) {
        std::rotate(rightCones.begin(), rightCones.begin() + 1, rightCones.end());
    }
    if (!gateSegments_.empty()) {
        std::rotate(gateSegments_.begin(), gateSegments_.begin() + 1, gateSegments_.end());
    }
}

void World::reset() {
    totalTime = 0.0;
    totalDistance = 0.0;
    lapCount = 0;
    insideLastCheckpoint_ = false;

    if (mission_.allowRegeneration && regenTrack) {
        std::printf("Regenerating track due to cone collision.\n");
        if (mission_.trackSource == fsai::sim::TrackSource::kRandom) {
            mission_.track = {};
        }
        trackState_ = buildTrackState();
    } else {
        std::printf("Resetting simulation without regenerating track.\n");
    }

    configureTrackState(trackState_);
    configureMissionRuntime();

    initializeVehiclePose();

    const float initDx = spawnState_.transform.position.x - lastCheckpoint.x;
    const float initDz = spawnState_.transform.position.z - lastCheckpoint.z;
    const float initDist = std::sqrt(initDx * initDx + initDz * initDz);
    insideLastCheckpoint_ = initDist < config.lapCompletionThreshold;
    coneDetections.clear();
    vehicleResetPending_ = true;
}
