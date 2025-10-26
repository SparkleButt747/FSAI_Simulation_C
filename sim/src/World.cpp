#include <algorithm>
#include <cmath>
#include <cstdio>
#include "fsai_clock.h"
#include "World.hpp"

static Vector3 transformToVector3(const Transform& t) {
    return {t.position.x, t.position.y, t.position.z};
}

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

void World::init(const char* yamlFilePath) {
    // Generate track data
    PathConfig pathConfig;
    int nPoints = pathConfig.resolution;
    PathGenerator pathGen(pathConfig);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(pathConfig, path);

    // Store checkpoint and cone data
    checkpointPositions.clear();
    leftCones.clear();
    rightCones.clear();
    for (const auto& cp : track.checkpoints) {
        checkpointPositions.push_back(transformToVector3(cp));
    }
    for (const auto& lc : track.leftCones) {
        leftCones.push_back(transformToVector3(lc));
    }
    for (const auto& rc : track.rightCones) {
        rightCones.push_back(transformToVector3(rc));
    }
    if (!track.checkpoints.empty()) {
        lastCheckpoint = transformToVector3(track.checkpoints.back());
    } else {
        lastCheckpoint = {0, 0, 0};
    }

    // Initialize simulation variables
    totalTime = 0.0;
    totalDistance = 0.0;
    lapCount = 0;
    deltaTime = 0.0;

    // Controller configuration
    config.collisionThreshold = 2.5f;
    config.coneCollisionThreshold = 0.5f;
    config.lapCompletionThreshold = 0.1f;

    // Use racing algorithm by default
    useController = 1;

    // Regenerate track after cone collision
    regenTrack = 1;

    // Racing algorithm configuration
    racingConfig.speedLookAheadSensitivity = 0.7f;
    racingConfig.steeringLookAheadSensitivity = 0.1f;
    racingConfig.accelerationFactor = 0.002f;

    // Make sure car starts on the track
    float startX = checkpointPositions[0].x;
    float startZ = checkpointPositions[0].z;
    float nextX = checkpointPositions[10].x;
    float nextZ = checkpointPositions[10].z;
    Vector2 zeroVector{0,0};
    Vector2 startVector{nextX - startX, nextZ - startZ};
    float startYaw = Vector2_SignedAngle(zeroVector, startVector) * M_PI/180.0f;

    // Initialize car model, state, and transform
    VehicleParam vp = VehicleParam::loadFromFile(yamlFilePath);
    carModel = DynamicBicycle(vp);
    carInput = VehicleInput(0.0, 0.0, 0.0);
    carState = VehicleState(Eigen::Vector3d(startX, startZ, 0.0), startYaw,
                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero());
    carTransform.position.x = static_cast<float>(carState.position.x());
    carTransform.position.y = 0.5f;
    carTransform.position.z = static_cast<float>(carState.position.y());
    carTransform.yaw = static_cast<float>(carState.yaw);

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
    std::printf("Next Checkpoint: (%f, %f)\n", checkpointPositions[0].x, checkpointPositions[0].z);

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

    for (const auto& cone : leftCones) {
        float cdx = carTransform.position.x - cone.x;
        float cdz = carTransform.position.z - cone.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        if (cdist < config.coneCollisionThreshold) {
            std::printf("Collision with a cone detected.\n");
            reset();
            return false;
        }
    }
    for (const auto& cone : rightCones) {
        float cdx = carTransform.position.x - cone.x;
        float cdz = carTransform.position.z - cone.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        if (cdist < config.coneCollisionThreshold) {
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
    if (regenTrack) {
        totalTime = 0.0;
        totalDistance = 0.0;

        std::printf("Regenerating track due to cone collision.\n");

        PathConfig pathConfig;
        int nPoints = pathConfig.resolution;
        PathGenerator pathGen(pathConfig);
        PathResult path = pathGen.generatePath(nPoints);
        TrackGenerator trackGen;
        TrackResult track = trackGen.generateTrack(pathConfig, path);

        checkpointPositions.clear();
        leftCones.clear();
        rightCones.clear();
        for (const auto& cp : track.checkpoints) {
            checkpointPositions.push_back(transformToVector3(cp));
        }
        for (const auto& lc : track.leftCones) {
            leftCones.push_back(transformToVector3(lc));
        }
        for (const auto& rc : track.rightCones) {
            rightCones.push_back(transformToVector3(rc));
        }
        if (!track.checkpoints.empty()) {
            lastCheckpoint = transformToVector3(track.checkpoints.back());
        } else {
            lastCheckpoint = {0,0,0};
        }

        carInput = VehicleInput(0.0, 0.0, 0.0);

        float startX = checkpointPositions[0].x;
        float startZ = checkpointPositions[0].z;
        float nextX = checkpointPositions[10].x;
        float nextZ = checkpointPositions[10].z;
        Vector2 zeroVector{0,0};
        Vector2 startVector{nextX - startX, nextZ - startZ};
        float startYaw = Vector2_SignedAngle(zeroVector, startVector) * M_PI/180.0f;

        carState = VehicleState(Eigen::Vector3d(startX, startZ, 0.0), startYaw,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
        carTransform.position.x = static_cast<float>(carState.position.x());
        carTransform.position.y = 0.5f;
        carTransform.position.z = static_cast<float>(carState.position.y());
        carTransform.yaw = static_cast<float>(carState.yaw);
    } else {
        carInput = VehicleInput(0.0, 0.0, 0.0);
        totalTime = 0.0;
        totalDistance = 0.0;

        std::printf("Resetting simulation without regenerating track.\n");

        float startX = checkpointPositions[0].x;
        float startZ = checkpointPositions[0].z;
        float nextX = checkpointPositions[10].x;
        float nextZ = checkpointPositions[10].z;
        Vector2 zeroVector{0,0};
        Vector2 startVector{nextX - startX, nextZ - startZ};
        float startYaw = Vector2_SignedAngle(zeroVector, startVector) * M_PI/180.0f;

        carState = VehicleState(Eigen::Vector3d(startX, startZ, 0.0), startYaw,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
        carTransform.position.x = static_cast<float>(carState.position.x());
        carTransform.position.y = 0.5f;
        carTransform.position.z = static_cast<float>(carState.position.y());
        carTransform.yaw = static_cast<float>(carState.yaw);
    }
}

