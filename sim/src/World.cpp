#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <utility>
#include "fsai_clock.h"
#include "World.hpp"
#include "sim/cone_constants.hpp"

namespace {

using fsai::sim::kLargeConeMassKg;
using fsai::sim::kLargeConeRadiusMeters;
using fsai::sim::kSmallConeMassKg;
using fsai::sim::kSmallConeRadiusMeters;

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

    if (mission_.trackSource == fsai::sim::TrackSource::kRandom &&
        mission_.track.checkpoints.empty()) {
        mission_.track = generateRandomTrack();
    }

    if (mission_.track.checkpoints.empty()) {
        throw std::runtime_error("MissionDefinition did not provide any checkpoints");
    }

    configureTrackState(mission_.track);

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
    totalTime += dt;

    if (checkpointPositions.empty()) {
        std::printf("No checkpoints available. Resetting simulation.\n");
        reset();
        return;
    }

    if (hasSvcuCommand_) {
        throttleInput = lastSvcuThrottle_;
        brakeInput = lastSvcuBrake_;
        steeringAngle = lastSvcuSteer_;
    }

    const double netAcc = static_cast<double>(throttleInput - brakeInput);
    carInput.acc = netAcc;
    carInput.delta = steeringAngle;

    carModel.updateState(carState, carInput, dt);

    carTransform.position.x = static_cast<float>(carState.position.x());
    carTransform.position.z = static_cast<float>(carState.position.y());
    carTransform.yaw = static_cast<float>(carState.yaw);

    totalDistance += carState.velocity.x() * dt;

    wheelsInfo_ = carModel.getWheelSpeeds(carState, carInput);

    hasSvcuCommand_ = false;

    if (!detectCollisions()) {
        return;
    }

    telemetry();
}

bool World::detectCollisions() {
    float dx = carTransform.position.x - checkpointPositions[0].x;
    float dz = carTransform.position.z - checkpointPositions[0].z;
    float distToNext = std::sqrt(dx * dx + dz * dz);
    if (distToNext < config.collisionThreshold) {
        moveNextCheckpointToLast();
    }

    dx = carTransform.position.x - lastCheckpoint.x;
    dz = carTransform.position.z - lastCheckpoint.z;
    float distToLast = std::sqrt(dx * dx + dz * dz);
    if (distToLast < config.lapCompletionThreshold) {
        if (lapCount > 0) {
            std::printf("Lap Completed. Time: %.2f s, Distance: %.2f, Lap: %d\n",
                       totalTime, totalDistance, lapCount);
        }
        lapCount++;
        totalTime = 0.0;
        totalDistance = 0.0;
    }

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
                     fsai_clock_now(), totalTime, totalDistance, lapCount);
}

void World::configureTrackState(const fsai::sim::TrackData& track) {
    checkpointPositions.clear();
    startCones.clear();
    leftCones.clear();
    rightCones.clear();

    for (const auto& cp : track.checkpoints) {
        checkpointPositions.push_back(transformToVector3(cp));
    }
    for (const auto& sc : track.startCones) {
        startCones.push_back(makeCone(sc, ConeType::Start));
    }
    for (const auto& lc : track.leftCones) {
        leftCones.push_back(makeCone(lc, ConeType::Left));
    }
    for (const auto& rc : track.rightCones) {
        rightCones.push_back(makeCone(rc, ConeType::Right));
    }

    if (!track.checkpoints.empty()) {
        lastCheckpoint = transformToVector3(track.checkpoints.back());
    } else {
        lastCheckpoint = {0.0f, 0.0f, 0.0f};
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
    if (!leftCones.empty()) {
        std::rotate(leftCones.begin(), leftCones.begin() + 1, leftCones.end());
    }
    if (!rightCones.empty()) {
        std::rotate(rightCones.begin(), rightCones.begin() + 1, rightCones.end());
    }
}

void World::reset() {
    carInput = VehicleInput(0.0, 0.0, 0.0);
    totalTime = 0.0;
    totalDistance = 0.0;

    if (mission_.allowRegeneration && regenTrack) {
        std::printf("Regenerating track due to cone collision.\n");
        if (mission_.trackSource == fsai::sim::TrackSource::kRandom) {
            mission_.track = generateRandomTrack();
        }
        configureTrackState(mission_.track);
    } else {
        std::printf("Resetting simulation without regenerating track.\n");
    }

    initializeVehiclePose();
}

