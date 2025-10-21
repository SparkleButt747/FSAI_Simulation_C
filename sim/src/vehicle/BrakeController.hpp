#pragma once
#include "VehicleParam.hpp"
#include <algorithm>

class BrakeController {
public:
  BrakeController() = default;
  explicit BrakeController(const BrakeParam& p) : P_(p) { normalize_(); }

  void setParam(const BrakeParam& p) { P_ = p; normalize_(); }

  BrakeRequest prepare(double command01, double speed_mps) const {
    double cmd = std::clamp(command01, 0.0, 1.0);
    double regen_cmd = 0.0;
    if (speed_mps >= P_.min_regen_speed && P_.regen_fraction > 0.0)
      regen_cmd = std::clamp(cmd * P_.regen_fraction, 0.0, 1.0);
    double target = cmd * std::max(0.0, P_.max_force);
    return {cmd, regen_cmd, target};
  }

  BrakeStatus finalize(const BrakeRequest& req, double regen_force_available) const {
    double regen = std::max(0.0, regen_force_available);
    double mech  = std::max(0.0, req.target_force - regen);
    double front = mech * P_.front_bias;
    double rear  = mech * P_.rear_bias;
    return {req.command, req.regen_command, regen, front, rear, mech};
  }

private:
  void normalize_() {
    double sum = std::max(1e-6, P_.front_bias + P_.rear_bias);
    P_.front_bias = std::clamp(P_.front_bias / sum, 0.0, 1.0);
    P_.rear_bias  = 1.0 - P_.front_bias;
  }

  BrakeParam P_{};
};
