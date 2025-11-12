#pragma once
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "VehicleParam.hpp"
#include "WheelsInfo.h"

class VehicleModel {
protected:
  VehicleParam param_{};

public:
  VehicleModel() = default;
  explicit VehicleModel(const VehicleParam& p) : param_(p) {}
  virtual ~VehicleModel() = default;

  virtual void updateState(VehicleState& state,
                           const VehicleInput& input,
                           double dt);

  // Clamp/correct raw inputs to configured ranges (with sanity).
  void validateInput(VehicleInput& input) const;

  // Optional post-step clamp/corrections (hook point).
  void validateState(VehicleState& state) const;

  // Slip angle utility (front if isFront=true, rear otherwise).
  double getSlipAngle(const VehicleState& x,
                      const VehicleInput& u,
                      bool isFront) const;

  // Basic kinematic wheel RPM + steering echo (checked).
  WheelsInfo getWheelSpeeds(const VehicleState& state,
                            const VehicleInput& input) const;

  const VehicleParam& param() const { return param_; }
};
