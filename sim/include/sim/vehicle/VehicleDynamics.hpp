#pragma once

#include "DynamicBicycle.hpp"
#include "Transform.h"
#include "VehicleInput.hpp"
#include "VehicleState.hpp"
#include "WheelsInfo.h"

class VehicleDynamics {
public:
    VehicleDynamics();
    explicit VehicleDynamics(const VehicleParam& param);

    void setCommand(float throttle, float brake, float steer);
    void step(double dt);

    void setState(const VehicleState& state, const Transform& transform);
    void resetInput();

    const VehicleState& state() const { return state_; }
    const Transform& transform() const { return transform_; }
    const WheelsInfo& wheelsInfo() const { return wheelsInfo_; }
    const DynamicBicycle& model() const { return model_; }

private:
    void updateTransformFromState();

    DynamicBicycle model_;
    VehicleState state_{Eigen::Vector3d::Zero(), 0.0,
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero()};
    VehicleInput input_{0.0, 0.0, 0.0};
    Transform transform_{};
    WheelsInfo wheelsInfo_{WheelsInfo_default()};
    float throttleCommand_{0.0f};
    float brakeCommand_{0.0f};
    float steerCommand_{0.0f};
};

