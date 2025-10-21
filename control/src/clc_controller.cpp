#include "control/clc_controller.hpp"

#include "sim/integration/path_truth.hpp"

#include <cmath>
#include <Eigen/Dense>

#include "budget.h"
#include "fsai_clock.h"

namespace {

Vector3 StartPointFromTruth(const fsai::integration::PathTruth* truth, size_t index) {
    if (!truth || truth->centerline.empty()) {
        return Vector3{0.0f, 0.0f, 0.0f};
    }
    if (index >= truth->centerline.size()) {
        index = truth->centerline.size() - 1;
    }
    return truth->centerline[index];
}

float ComputeInitialYaw(const Vector3& start, const Vector3& next) {
    const float dx = next.x - start.x;
    const float dz = next.z - start.z;
    if (std::fabs(dx) < 1e-3f && std::fabs(dz) < 1e-3f) {
        return 0.0f;
    }
    return std::atan2(dz, dx);
}

}  // namespace

void CarController_Init(CarController* controller,
                        const char* yamlFilePath,
                        const fsai::integration::PathTruth* truth) {
    controller->pathTruth = truth;
    controller->checkpointPositions = nullptr;
    controller->nCheckpoints = 0;

    controller->totalTime = 0.0;
    controller->totalDistance = 0.0;
    controller->lapCount = 0;
    controller->deltaTime = 0.0;
    controller->startTimeNs = 0;
    controller->lapStartTimeNs = 0;
    controller->lastTickNs = 0;

    controller->config.collisionThreshold = 2.5f;
    controller->config.conecollisionThreshold = 0.5f;
    controller->config.lap_completion_collisionThreshold = 0.1f;

    controller->useRacingAlgorithm = 1;
    controller->regenTrack = 0;

    controller->racingConfig.speedLookAheadSensitivity = 0.7f;
    controller->racingConfig.steeringLookAheadSensitivity = 0.1f;
    controller->racingConfig.accelerationFactor = 0.002f;

    const Vector3 startPoint = StartPointFromTruth(truth, 0);
    const Vector3 nextPoint = StartPointFromTruth(truth, 1);
    const float startYaw = ComputeInitialYaw(startPoint, nextPoint);

    VehicleParam vp = VehicleParam::loadFromFile(yamlFilePath);
    controller->carModel = DynamicBicycle(vp);
    controller->carInput = VehicleInput(0.0, 0.0, 0.0);
    controller->carState = VehicleState(Eigen::Vector3d(startPoint.x, startPoint.z, 0.0),
                                        startYaw,
                                        Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d::Zero());

    const uint64_t now_ns = fsai_clock_now();
    controller->startTimeNs = now_ns;
    controller->lapStartTimeNs = now_ns;
    controller->lastTickNs = now_ns;
    controller->carState.timestampNs = now_ns;

    controller->carTransform.position.x = static_cast<float>(controller->carState.position.x());
    controller->carTransform.position.y = 0.5f;
    controller->carTransform.position.z = static_cast<float>(controller->carState.position.y());
    controller->carTransform.yaw = static_cast<float>(controller->carState.yaw);
}

void CarController_SetCheckpoints(CarController* controller,
                                  const Vector3* checkpoints,
                                  int count) {
    controller->checkpointPositions = checkpoints;
    controller->nCheckpoints = count;
}

FsaiControlCmd CarController_Update(CarController* controller, double dt, uint64_t now_ns) {
    FsaiControlCmd cmd{};
    cmd.t_ns = now_ns;

    if (now_ns < controller->lapStartTimeNs) {
        controller->lapStartTimeNs = now_ns;
    }

    const double dt_from_clock = (controller->lastTickNs > 0)
        ? fsai_clock_to_seconds(now_ns - controller->lastTickNs)
        : dt;
    controller->deltaTime = dt_from_clock > 0.0 ? dt_from_clock : dt;
    controller->totalTime = fsai_clock_to_seconds(now_ns - controller->lapStartTimeNs);
    controller->lastTickNs = now_ns;
    controller->carState.timestampNs = now_ns;

    if (controller->useRacingAlgorithm && controller->checkpointPositions && controller->nCheckpoints > 0) {
        Vector3 carVelocity = {static_cast<float>(controller->carState.velocity.x()),
                               static_cast<float>(controller->carState.velocity.y()),
                               static_cast<float>(controller->carState.velocity.z())};
        float carSpeed = Vector3_Magnitude(carVelocity);
        {
            fsai::time::ControlStageTimer planner_timer("planner");
            controller->lookaheadIndices = RacingAlgorithm_GetLookaheadIndices(
                controller->nCheckpoints,
                carSpeed,
                &controller->racingConfig);
            controller->throttleInput = RacingAlgorithm_GetThrottleInput(
                controller->checkpointPositions,
                controller->nCheckpoints,
                carSpeed,
                &controller->carTransform,
                &controller->racingConfig,
                controller->deltaTime);
            controller->steeringAngle = RacingAlgorithm_GetSteeringInput(
                controller->checkpointPositions,
                controller->nCheckpoints,
                carSpeed,
                &controller->carTransform,
                &controller->racingConfig,
                controller->deltaTime);
        }
    } else {
        controller->throttleInput = 0.0f;
        controller->steeringAngle = 0.0f;
    }

    controller->carInput.acc = controller->throttleInput;
    controller->carInput.delta = controller->steeringAngle;

    {
        fsai::time::ControlStageTimer model_timer("vehicle_model");
        controller->carModel.updateState(controller->carState, controller->carInput, controller->deltaTime);
    }

    controller->carTransform.position.x = static_cast<float>(controller->carState.position.x());
    controller->carTransform.position.z = static_cast<float>(controller->carState.position.y());
    controller->carTransform.yaw = static_cast<float>(controller->carState.yaw);

    const double vx = controller->carState.velocity.x();
    const double vy = controller->carState.velocity.y();
    const double vz = controller->carState.velocity.z();
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    controller->totalDistance += speed * controller->deltaTime;

    Telemetry_Update(controller->carState, controller->carTransform,
                     now_ns, controller->totalTime,
                     controller->totalDistance, controller->lapCount);

    cmd.steer_rad = controller->carInput.delta;
    cmd.throttle = controller->carInput.acc;
    cmd.brake = 0.0f;
    return cmd;
}

void CarController_Reset(CarController* controller, uint64_t now_ns) {
    const Vector3 startPoint = StartPointFromTruth(controller->pathTruth, 0);
    const Vector3 nextPoint = StartPointFromTruth(controller->pathTruth, 1);
    const float startYaw = ComputeInitialYaw(startPoint, nextPoint);

    controller->carInput = VehicleInput(0.0, 0.0, 0.0);
    controller->totalTime = 0.0;
    controller->totalDistance = 0.0;
    controller->startTimeNs = now_ns;
    controller->lapStartTimeNs = now_ns;
    controller->lastTickNs = now_ns;

    controller->carState = VehicleState(Eigen::Vector3d(startPoint.x, startPoint.z, 0.0),
                                        startYaw,
                                        Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d::Zero(),
                                        now_ns);

    controller->carTransform.position.x = static_cast<float>(controller->carState.position.x());
    controller->carTransform.position.y = 0.5f;
    controller->carTransform.position.z = static_cast<float>(controller->carState.position.y());
    controller->carTransform.yaw = static_cast<float>(controller->carState.yaw);
}

