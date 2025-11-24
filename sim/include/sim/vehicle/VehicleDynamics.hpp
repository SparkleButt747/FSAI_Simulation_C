#pragma once

#include "DynamicBicycle.hpp"
#include "Transform.h"
#include "VehicleInput.hpp"
#include "VehicleState.hpp"
#include "WheelsInfo.h"
#include "architecture/IVehicleDynamics.hpp"

class VehicleDynamics : public fsai::vehicle::IVehicleDynamics {
public:
    VehicleDynamics();
    explicit VehicleDynamics(const VehicleParam& param);

    void setCommand(float throttle, float brake, float steer);
    void set_command(float throttle, float brake, float steer) override {
        setCommand(throttle, brake, steer);
    }
    void step(double dt) override;

    void setState(const VehicleState& state, const Transform& transform);
    void set_state(const VehicleState& state, const Transform& transform) override {
        setState(state, transform);
    }
    void resetInput();
    void reset_input() override { resetInput(); }

    const VehicleState& state() const override { return state_; }
    const Transform& transform() const override { return transform_; }
    const WheelsInfo& wheels_info() const override { return wheelsInfo_; }
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

