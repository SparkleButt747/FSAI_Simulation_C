#pragma once
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "VehicleParam.hpp"
#include "WheelsInfo.h"
#include <algorithm>
#include <cmath>

class VehicleModel {
protected:
    VehicleParam param_;
public:
    VehicleModel() = default;
    explicit VehicleModel(const VehicleParam& p) : param_(p) {}
    virtual ~VehicleModel() = default;

    virtual void updateState(VehicleState& state, const VehicleInput& input, double dt);
    void validateState(VehicleState& state) const;
    void validateInput(VehicleInput& input) const;
    double getSlipAngle(const VehicleState& x, const VehicleInput& u, bool isFront) const;
    WheelsInfo getWheelSpeeds(const VehicleState& state, const VehicleInput& input) const;
    const VehicleParam& param() const { return param_; }
};

class DynamicBicycle : public VehicleModel {
public:
    using VehicleModel::VehicleModel;
    struct Forces { double Fx; double FyF; double FyR; };
    Forces computeForces(const VehicleState& state, const VehicleInput& input) const;
    void updateState(VehicleState& state, const VehicleInput& input, double dt) override;
    static double calculateMagnitude(double x, double y);
};
