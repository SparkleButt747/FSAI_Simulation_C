#include "VehicleModel.hpp"
#include "SanityChecks.hpp"
#include <algorithm>
#include <cmath>

using fsai::sim::sanity::checkInput;
using fsai::sim::sanity::checkState;
using fsai::sim::sanity::checkWheels;
using fsai::sim::sanity::fail;

static constexpr double kPi = 3.141592653589793;
static constexpr double kHalfPi = 1.5707963267948966;

// Low-speed helpers reused here
namespace {
inline double smoothBlend01(double t) { t = std::clamp(t, 0.0, 1.0); return t*t*(3.0 - 2.0*t); }
inline double lowSpeedBlend(double v) {
  const double a=0.6, b=1.8, span=std::max(1e-6, b-a);
  if (v<=a) return 0.0; if (v>=b) return 1.0; return smoothBlend01((v-a)/span);
}
inline double regularizedLongitudinal(double vx, double blend) {
  const double min_mag = 0.35 * (0.5 + 0.5 * blend);
  const double s = (vx>=0.0)?1.0:-1.0;
  return s * std::max(std::abs(vx), min_mag);
}
} // namespace

void VehicleModel::updateState(VehicleState&, const VehicleInput&, double) {
  // Base: no-op (child models implement)
}

void VehicleModel::validateInput(VehicleInput& input) const {
  // clamp to configured ranges
  input.acc   = std::clamp(input.acc,   param_.input_ranges.acc.min,   param_.input_ranges.acc.max);
  input.vel   = std::clamp(input.vel,   param_.input_ranges.vel.min,   param_.input_ranges.vel.max);
  input.delta = std::clamp(input.delta, param_.input_ranges.delta.min, param_.input_ranges.delta.max);

  // and assert it's sane
  checkInput(param_, input, "VehicleModel::validateInput");
}

void VehicleModel::validateState(VehicleState& state) const {
  // Optional place to enforce invariants (e.g., z=0)
  if (!std::isfinite(state.position.z())) state.position.z() = 0.0;
  checkState(param_, state, "VehicleModel::validateState");
}

double VehicleModel::getSlipAngle(const VehicleState& x,
                                  const VehicleInput& u,
                                  bool isFront) const {
  const double vx_raw = x.velocity.x();
  const double vy     = x.velocity.y();
  const double r      = x.rotation.z();
  const double lf     = param_.kinematic.l_F;
  const double lr     = param_.kinematic.l_R;

  const double speed  = std::sqrt(vx_raw * vx_raw + vy * vy);
  const double blend  = lowSpeedBlend(speed);

  // Regularize longitudinal component (already speed-aware)
  double vx_eff = regularizedLongitudinal(vx_raw, blend);

  // Extra denominator padding tied to lateral content and low-speed weight
  const double lat_mag   = std::abs(vy) + (isFront ? std::abs(lf * r) : std::abs(lr * r));
  const double denom_pad = (1.0 - blend) * (0.2 + 0.6 * std::min(1.0, lat_mag)); // meters/sec
  const double denom     = std::abs(vx_eff) + denom_pad; // never ~0

  // Numerator
  double num = isFront ? (vy + lf * r) : (vy - lr * r);

  // Raw slip and steering subtraction for front
  double alpha = std::atan2(num, denom);
  if (isFront) alpha -= u.delta;

  // Soften toward zero at low speed (noisy/undefined slip when nearly stopped)
  const double soften = 0.35 + 0.65 * blend; // at standstill ~0.35, at speed ~1.0
  alpha *= soften;

  // Avoid ever hitting exactly ±π/2 so sanity check doesn’t trip on rounding
  const double lim = kHalfPi - 1e-4;
  return std::clamp(alpha, -lim, lim);
}


WheelsInfo VehicleModel::getWheelSpeeds(const VehicleState& state,
                                        const VehicleInput& input) const {
  checkState(param_, state, "VehicleModel::getWheelSpeeds(state)");
  checkInput(param_, input, "VehicleModel::getWheelSpeeds(input)");

  WheelsInfo wi = WheelsInfo_default();
  const double wheelCirc = 2.0 * kPi * param_.tire.radius;
  const double rpm = (wheelCirc > 1e-12) ? (state.velocity.x() / wheelCirc) * 60.0 : 0.0;

  wi.lf_speed = wi.rf_speed = wi.lb_speed = wi.rb_speed = static_cast<float>(rpm);
  wi.steering = static_cast<float>(input.delta);

  checkWheels(param_, wi, state, "VehicleModel::getWheelSpeeds(out)");
  return wi;
}
