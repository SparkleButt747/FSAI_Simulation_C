#include "vehicle/VehicleDynamics.hpp"

VehicleDynamics::VehicleDynamics() : VehicleDynamics(VehicleParam()) {}

VehicleDynamics::VehicleDynamics(const VehicleParam& param) : model_(param) {}

void VehicleDynamics::setCommand(float throttle, float brake, float steer) {
    throttleCommand_ = throttle;
    brakeCommand_ = brake;
    steerCommand_ = steer;
}

void VehicleDynamics::step(double dt) {
    const double netAcc = static_cast<double>(throttleCommand_ - brakeCommand_);
    input_.acc = netAcc;
    input_.delta = static_cast<double>(steerCommand_);

    model_.updateState(state_, input_, dt);

    updateTransformFromState();

    wheelsInfo_ = model_.getWheelSpeeds(state_, input_);
}

void VehicleDynamics::setState(const VehicleState& state, const Transform& transform) {
    state_ = state;
    transform_ = transform;
    updateTransformFromState();
}

void VehicleDynamics::resetInput() {
    throttleCommand_ = 0.0f;
    brakeCommand_ = 0.0f;
    steerCommand_ = 0.0f;
    input_ = VehicleInput(0.0, 0.0, 0.0);
}

void VehicleDynamics::updateTransformFromState() {
    transform_.position.x = static_cast<float>(state_.position.x());
    transform_.position.z = static_cast<float>(state_.position.y());
    transform_.yaw = static_cast<float>(state_.yaw);
}

