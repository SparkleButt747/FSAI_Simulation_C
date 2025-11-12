#include "BrakeController.hpp"
#include "SanityChecks.hpp"
#include <algorithm>
#include <cmath>

using fsai::sim::sanity::fail;

BrakeController::BrakeController() = default;

BrakeController::BrakeController(const BrakeParam& p) : P_(p) {
  normalize_();
}

void BrakeController::setParam(const BrakeParam& p) {
  P_ = p;
  normalize_();
}

BrakeRequest BrakeController::prepare(double command01, double speed_mps) const {
  if (!std::isfinite(command01)) fail("BrakeController::prepare", "command01 !finite", command01);
  if (!std::isfinite(speed_mps)) fail("BrakeController::prepare", "speed_mps !finite", speed_mps);

  const double cmd = std::clamp(command01, 0.0, 1.0);

  double regen_cmd = 0.0;
  if (speed_mps >= P_.min_regen_speed && P_.regen_fraction > 0.0) {
    regen_cmd = std::clamp(cmd * P_.regen_fraction, 0.0, 1.0);
  }

  const double target = cmd * std::max(0.0, P_.max_force);

  if (!(regen_cmd >= 0.0 && regen_cmd <= 1.0)) fail("BrakeController::prepare", "regen_cmd out of [0,1]", regen_cmd);
  if (!(target >= 0.0 && std::isfinite(target))) fail("BrakeController::prepare", "target_force invalid", target);

  return { cmd, regen_cmd, target };
}

BrakeStatus BrakeController::finalize(const BrakeRequest& req, double regen_force_available) const {
  if (!std::isfinite(regen_force_available) || regen_force_available < 0.0)
    fail("BrakeController::finalize", "regen_force_available invalid", regen_force_available);

  const double regen = std::max(0.0, regen_force_available);
  const double mech  = std::max(0.0, req.target_force - regen);
  const double front = mech * P_.front_bias;
  const double rear  = mech * P_.rear_bias;

  const double total_mech = front + rear;

  if (front < -1e-9 || rear < -1e-9) fail("BrakeController::finalize", "negative mech split", front + rear);
  if (std::abs(total_mech - mech) > 1e-3 * std::max(1.0, mech))
    fail("BrakeController::finalize", "mech split mismatch", total_mech - mech);

  return { req.command, req.regen_command, regen, front, rear, total_mech };
}

void BrakeController::normalize_() {
  const double sum = std::max(1e-6, P_.front_bias + P_.rear_bias);
  P_.front_bias = std::clamp(P_.front_bias / sum, 0.0, 1.0);
  P_.rear_bias  = 1.0 - P_.front_bias;

  if (P_.max_force < 0.0) P_.max_force = 0.0;
  if (P_.regen_fraction < 0.0) P_.regen_fraction = 0.0;
}
