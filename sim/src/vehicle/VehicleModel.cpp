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
  const double vx = x.velocity.x();
  const double vy = x.velocity.y();
  const double r  = x.rotation.z();

  const double lf = param_.kinematic.l_F;
  const double lr = param_.kinematic.l_R;

  // Speed & blend in [0,1] for low-speed handling
  const double speed = std::hypot(vx, vy);
  const double blend = lowSpeedBlend(speed);          // 0..1 (you already have this)

  // Numerator: lateral content at axle
  const double num = isFront ? (vy + lf * r) : (vy - lr * r);

  // --- Key idea: denominator as max of three stabilizers ---
  // 1) True longitudinal magnitude
  const double D_vx = std::abs(vx);

  // 2) A speed-aware floor (larger when nearly stopped)
  //    at standstill ~ v_floor_hi, at speed ~ v_floor_lo
  const double v_floor_hi = 1.20;   // m/s  (aggressive floor at standstill)
  const double v_floor_lo = 0.05;   // m/s  (just avoids 0/0 at speed)
  const double D_floor = v_floor_lo * blend + v_floor_hi * (1.0 - blend);

  // 3) A proportional term to |num| to give a *hard analytic* bound on |alpha|
  //    If D >= lambda * |num| then |alpha| = atan(|num|/D) <= atan(1/lambda).
  //    We allow a speed-varying bound: lo at very low speed, hi at speed.
  const double alpha_max_hi = 1.2217304763960306; // 70 deg in rad at speed
  const double alpha_max_lo = 0.5235987755982989; // 30 deg in rad near stop
  const double alpha_max    = alpha_max_hi * blend + alpha_max_lo * (1.0 - blend);
  const double lambda       = 1.0 / std::tan(alpha_max); // ensures |alpha|<=alpha_max
  const double D_prop       = lambda * std::abs(num);

  // Final denominator
  const double D = std::max({D_vx, D_floor, D_prop});

  double alpha = std::atan2(num, D);
  if (isFront) alpha -= u.delta;    // front axle steer subtraction

  // Small numeric cushion under the target bound
  const double lim = alpha_max - 1e-4;
  if (alpha >  lim) alpha =  lim;
  if (alpha < -lim) alpha = -lim;

  return alpha;
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
