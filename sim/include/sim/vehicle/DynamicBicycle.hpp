#pragma once
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "VehicleParam.hpp"
#include "WheelsInfo.h"
#include "EVPowertrain.hpp"
#include "BrakeController.hpp"
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

  void updateState(VehicleState& state, const VehicleInput& input, double dt) override;
  static double calculateMagnitude(double x, double y);

  const PowertrainStatus& lastPowertrainStatus() const { return last_pt_status_; }
  const BrakeStatus& lastBrakeStatus() const { return last_brake_status_; }

private:
  // Actuator states (first-order lags)
  mutable double thr_eff_{0.0};
  mutable double brk_eff_{0.0};

  // Runtime powertrain/brakes
  mutable EVMotorPowertrain pt_;
  mutable BrakeController   br_;
  mutable PowertrainStatus  last_pt_status_{};
  mutable BrakeStatus       last_brake_status_{};

  Forces computeForces(const VehicleState& state, const VehicleInput& input, double dt) const;

    // helpers
    static inline double clamp(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
    static inline double clamp01(double v){ return v<0?0:(v>1?1:v); }
};
