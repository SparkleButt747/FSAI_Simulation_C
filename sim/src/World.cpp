#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <utility>
#include "checkpoint_matching.hpp"
#include "fsai_clock.h"
#include "World.hpp"
#include "sim/cone_constants.hpp"

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
        case ConeType::Left:
        case ConeType::Right:
            cone.radius = kSmallConeRadiusMeters;
            cone.mass = kSmallConeMassKg;
            break;
    }
    return cone;
}

}  // namespace

bool World::computeRacingControl(double dt, float& throttle_out, float& steering_out) {
    if (!useController || checkpointPositions.empty()) {
        return false;
    }

    Vector3 carVelocity{static_cast<float>(carState.velocity.x()),
                        static_cast<float>(carState.velocity.y()),
                        static_cast<float>(carState.velocity.z())};
    const float carSpeed = Vector3_Magnitude(carVelocity);
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

void World::setSvcuCommand(float throttle, float brake, float steer) {
    lastSvcuThrottle_ = throttle;
    lastSvcuBrake_ = brake;
    lastSvcuSteer_ = steer;
    hasSvcuCommand_ = true;
}

void World::init(const char* yamlFilePath, fsai::sim::MissionDefinition mission) {
    mission_ = std::move(mission);

    const bool has_boundaries = !mission_.track.leftCones.empty() &&
                                !mission_.track.rightCones.empty();
    const bool has_checkpoints = !mission_.track.checkpoints.empty();

    if (mission_.trackSource == fsai::sim::TrackSource::kRandom &&
        !has_boundaries && !has_checkpoints) {
        mission_.track = generateRandomTrack();
    }

    if (!has_checkpoints &&
        (mission_.track.leftCones.empty() || mission_.track.rightCones.empty())) {
        throw std::runtime_error("MissionDefinition must provide track boundaries or checkpoints");
    }

    configureTrackState(mission_.track);
    configureMissionRuntime();

    totalTime = 0.0;
    totalDistance = 0.0;
    lapCount = 0;
    deltaTime = 0.0;

    config.collisionThreshold = 1.75f;
    config.vehicleCollisionRadius = 0.5f - kSmallConeRadiusMeters;
    config.lapCompletionThreshold = 0.2f;

    useController = 1;
    regenTrack = mission_.allowRegeneration ? 1 : 0;

    racingConfig.speedLookAheadSensitivity = 0.5f;
    racingConfig.steeringLookAheadSensitivity = 0;
    racingConfig.accelerationFactor = 0.0019f;

    VehicleParam vp = VehicleParam::loadFromFile(yamlFilePath);
    carModel = DynamicBicycle(vp);
    carInput = VehicleInput(0.0, 0.0, 0.0);

    initializeVehiclePose();

    wheelsInfo_ = WheelsInfo_default();
    hasSvcuCommand_ = false;
    lastSvcuThrottle_ = 0.0f;
    lastSvcuBrake_ = 0.0f;
    lastSvcuSteer_ = 0.0f;
}

void World::update(double dt) {
    deltaTime = dt;

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

    const double netAcc = static_cast<double>(throttleInput - brakeInput);
    carInput.acc = netAcc;
    carInput.delta = steeringAngle;

    carModel.updateState(carState, carInput, dt);

    carTransform.position.x = static_cast<float>(carState.position.x());
    carTransform.position.z = static_cast<float>(carState.position.y());
    carTransform.yaw = static_cast<float>(carState.yaw);

    const Vector2 currentPos{carTransform.position.x, carTransform.position.z};
    const bool crossedGate = crossesCurrentGate(prevCarPos_, currentPos);

    const Eigen::Vector2d velocity2d(carState.velocity.x(), carState.velocity.y());
    if (!missionState_.mission_complete()) {
        totalDistance += velocity2d.norm() * dt;
    }

    wheelsInfo_ = carModel.getWheelSpeeds(carState, carInput);

    if (!detectCollisions(crossedGate)) {
        return;
    }

    prevCarPos_ = currentPos;

    updateStraightLineProgress();
    handleMissionCompletion();

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

    return true;
}

void World::telemetry() const {
    Telemetry_Update(carState, carTransform,
                     fsai_clock_now(), totalTime, totalDistance, lapCount,
                     missionState_);
}

void World::configureTrackState(const fsai::sim::TrackData& track) {
    checkpointPositions.clear();
    startCones.clear();
    leftCones.clear();
    rightCones.clear();
    gateLeftCones_.clear();
    gateRightCones_.clear();

    for (const auto& sc : track.startCones) {
        startCones.push_back(makeCone(sc, ConeType::Start));
    }
    for (const auto& lc : track.leftCones) {
        leftCones.push_back(makeCone(lc, ConeType::Left));
    }
    for (const auto& rc : track.rightCones) {
        rightCones.push_back(makeCone(rc, ConeType::Right));
    }

    const std::vector<fsai::control::MatchedGate> matched_gates =
        fsai::control::MatchConesToGates(track.leftCones, track.rightCones);

    if (!matched_gates.empty()) {
        checkpointPositions.reserve(matched_gates.size());
        gateLeftCones_.reserve(matched_gates.size());
        gateRightCones_.reserve(matched_gates.size());
        for (const auto& gate : matched_gates) {
            gateLeftCones_.push_back(makeCone(gate.left, ConeType::Left));
            gateRightCones_.push_back(makeCone(gate.right, ConeType::Right));
            checkpointPositions.push_back(transformToVector3(gate.checkpoint));
        }
    } else {
        for (const auto& cp : track.checkpoints) {
            checkpointPositions.push_back(transformToVector3(cp));
        }
    }

    if (!checkpointPositions.empty()) {
        lastCheckpoint = checkpointPositions.back();
    } else {
        lastCheckpoint = {0.0f, 0.0f, 0.0f};
    }
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
    if (checkpointPositions.empty()) {
        carState = VehicleState(Eigen::Vector3d::Zero(), 0.0,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
        carTransform.position.x = 0.0f;
        carTransform.position.y = 0.5f;
        carTransform.position.z = 0.0f;
        carTransform.yaw = 0.0f;
        prevCarPos_ = {carTransform.position.x, carTransform.position.z};
        return;
    }

    const Vector3& start = checkpointPositions.front();
    const std::size_t lookaheadIndex = checkpointPositions.size() > 1
                                           ? std::min<std::size_t>(10, checkpointPositions.size() - 1)
                                           : 0;
    const Vector3& next = checkpointPositions[lookaheadIndex];
    Vector2 startVector{next.x - start.x, next.z - start.z};

    float startYaw = 0.0f;
    if (checkpointPositions.size() > 1) {
        Vector2 zeroVector{0.0f, 0.0f};
        startYaw = Vector2_SignedAngle(zeroVector, startVector) *
                   (std::numbers::pi_v<float> / 180.0f);
    }

    carState = VehicleState(
        Eigen::Vector3d(static_cast<double>(start.x), static_cast<double>(start.z), 0.0),
        startYaw, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    carTransform.position.x = static_cast<float>(carState.position.x());
    carTransform.position.y = 0.5f;
    carTransform.position.z = static_cast<float>(carState.position.y());
    carTransform.yaw = static_cast<float>(carState.yaw);
    prevCarPos_ = {carTransform.position.x, carTransform.position.z};
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

    const std::vector<Cone>& gateLeft = gateLeftCones_.empty() ? leftCones : gateLeftCones_;
    const std::vector<Cone>& gateRight = gateRightCones_.empty() ? rightCones : gateRightCones_;

    if (gateLeft.empty() || gateRight.empty()) {
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

    const Vec2d left{static_cast<double>(gateLeft.front().position.x),
                     static_cast<double>(gateLeft.front().position.z)};
    const Vec2d right{static_cast<double>(gateRight.front().position.x),
                      static_cast<double>(gateRight.front().position.z)};

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
    if (!gateLeftCones_.empty() && !gateRightCones_.empty()) {
        std::rotate(gateLeftCones_.begin(), gateLeftCones_.begin() + 1, gateLeftCones_.end());
        std::rotate(gateRightCones_.begin(), gateRightCones_.begin() + 1, gateRightCones_.end());
    } else {
        if (!leftCones.empty()) {
            std::rotate(leftCones.begin(), leftCones.begin() + 1, leftCones.end());
        }
        if (!rightCones.empty()) {
            std::rotate(rightCones.begin(), rightCones.begin() + 1, rightCones.end());
        }
    }
}

void World::reset() {
    carInput = VehicleInput(0.0, 0.0, 0.0);
    totalTime = 0.0;
    totalDistance = 0.0;
    lapCount = 0;
    insideLastCheckpoint_ = false;

    if (mission_.allowRegeneration && regenTrack) {
        std::printf("Regenerating track due to cone collision.\n");
        if (mission_.trackSource == fsai::sim::TrackSource::kRandom) {
            mission_.track = generateRandomTrack();
        }
        configureTrackState(mission_.track);
    } else {
        std::printf("Resetting simulation without regenerating track.\n");
    }

    configureMissionRuntime();

    initializeVehiclePose();

    const float initDx = carTransform.position.x - lastCheckpoint.x;
    const float initDz = carTransform.position.z - lastCheckpoint.z;
    const float initDist = std::sqrt(initDx * initDx + initDz * initDz);
    insideLastCheckpoint_ = initDist < config.lapCompletionThreshold;
}

