#include "DynamicBicycle.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double kPiConst = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;

struct LowSpeedContext {
  double speed{0.0};
  double blend{0.0};
  double vx_effective{0.0};
};

inline double smoothUnit(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

LowSpeedContext evaluateLowSpeedContext(double vx, double vy) {
  const double speed = std::hypot(vx, vy);
  constexpr double kBlendStart = 0.4;
  constexpr double kBlendEnd = 2.0;
  double blend = 0.0;
  if (speed <= kBlendStart) {
    blend = 0.0;
  } else if (speed >= kBlendEnd) {
    blend = 1.0;
  } else {
    const double span = std::max(kBlendEnd - kBlendStart, 1e-6);
    blend = smoothUnit((speed - kBlendStart) / span);
  }

  const double min_mag = 0.18 + 0.14 * (1.0 - blend);
  const double sign = (vx >= 0.0) ? 1.0 : -1.0;
  const double vx_eff = sign * std::max(std::abs(vx), min_mag);

  return LowSpeedContext{speed, blend, vx_eff};
}

struct WheelLoads {
  double Fz_fl{0.0};
  double Fz_fr{0.0};
  double Fz_rl{0.0};
  double Fz_rr{0.0};

  double total() const { return Fz_fl + Fz_fr + Fz_rl + Fz_rr; }
};

WheelLoads computeWheelLoads(const VehicleParam& P, double Fdown, double ax, double ay) {
  const double m = P.inertia.m;
  const double g = P.inertia.g;
  const double Wtot = m * g + Fdown;
  const double Wf = Wtot * P.kinematic.w_front;
  const double Wr = Wtot - Wf;

  WheelLoads loads;
  loads.Fz_fl = 0.5 * Wf;
  loads.Fz_fr = 0.5 * Wf;
  loads.Fz_rl = 0.5 * Wr;
  loads.Fz_rr = 0.5 * Wr;

  const double lf = P.kinematic.l_F;
  const double lr = P.kinematic.l_R;
  const double L = lf + lr;
  const double h = (P.physics.h_cg > 1e-6) ? P.physics.h_cg : 0.25;
  const double tw = std::max(0.5, P.kinematic.axle_width);

  const double dF_long = (m * ax * h) / std::max(L, 1e-6);
  loads.Fz_fl -= 0.5 * dF_long;
  loads.Fz_fr -= 0.5 * dF_long;
  loads.Fz_rl += 0.5 * dF_long;
  loads.Fz_rr += 0.5 * dF_long;

  const double dF_lat = (m * ay * h) / tw;
  loads.Fz_fl += 0.5 * dF_lat;
  loads.Fz_rl += 0.5 * dF_lat;
  loads.Fz_fr -= 0.5 * dF_lat;
  loads.Fz_rr -= 0.5 * dF_lat;

  loads.Fz_fl = std::max(0.0, loads.Fz_fl);
  loads.Fz_fr = std::max(0.0, loads.Fz_fr);
  loads.Fz_rl = std::max(0.0, loads.Fz_rl);
  loads.Fz_rr = std::max(0.0, loads.Fz_rr);
  return loads;
}

double pacejkaLat(double slip, double B, double C, double D, double E) {
  return D * std::sin(C * std::atan(B * (1.0 - E) * slip + E * std::atan(B * slip)));
}

struct WheelForceState {
  double Fx{0.0};
  double Fy{0.0};
  double Fz{0.0};
};

std::pair<double, double> frictionCircleClip(double Fx, double Fy, double muFz, double& usage_out) {
  muFz = std::max(muFz, 1e-6);
  const double nx = Fx / muFz;
  const double ny = Fy / muFz;
  const double r = std::sqrt(nx * nx + ny * ny);
  usage_out = r;
  if (r <= 1.0) {
    return {Fx, Fy};
  }
  const double s = 1.0 / r;
  return {Fx * s, Fy * s};
}

inline double rollingResistance(double c_rr, double normal_force, double vx) {
  if (c_rr <= 0.0) return 0.0;
  const double base = c_rr * normal_force;
  if (std::abs(vx) < 1e-5) return 0.0;
  const double fade = std::tanh(std::abs(vx) / 0.6);
  return base * fade * ((vx >= 0.0) ? 1.0 : -1.0);
}

inline double lowSpeedDampingGain(double blend, double brake_level) {
  const double low_weight = 1.0 - blend;
  if (low_weight <= 1e-6) return 0.0;
  const double base = 5.0 * low_weight;
  const double brake = 4.0 * low_weight * std::clamp(brake_level, 0.0, 1.0);
  return base + brake;
}

} // namespace

// -----------------------------------------------------------------------------
// VehicleModel base utilities

void VehicleModel::updateState(VehicleState&, const VehicleInput&, double) {
}

void VehicleModel::validateState(VehicleState& state) const {
  (void)state;
}

void VehicleModel::validateInput(VehicleInput& input) const {
  input.acc = std::clamp(input.acc, param_.input_ranges.acc.min, param_.input_ranges.acc.max);
  input.vel = std::clamp(input.vel, param_.input_ranges.vel.min, param_.input_ranges.vel.max);
  input.delta = std::clamp(input.delta, param_.input_ranges.delta.min, param_.input_ranges.delta.max);
}

double VehicleModel::getSlipAngle(const VehicleState& x, const VehicleInput& u, bool isFront) const {
  const auto ctx = evaluateLowSpeedContext(x.velocity.x(), x.velocity.y());
  const double lf = param_.kinematic.l_F;
  const double lr = param_.kinematic.l_R;
  const double r = x.rotation.z();

  double slip;
  if (isFront) {
    slip = std::atan2(x.velocity.y() + lf * r, ctx.vx_effective) - u.delta;
  } else {
    slip = std::atan2(x.velocity.y() - lr * r, ctx.vx_effective);
  }
  slip *= ctx.blend;
  return std::clamp(slip, -kHalfPi, kHalfPi);
}

WheelsInfo VehicleModel::getWheelSpeeds(const VehicleState& state, const VehicleInput& input) const {
  WheelsInfo wheelSpeeds = WheelsInfo_default();
  const double wheel_circumference = 2.0 * kPiConst * param_.tire.radius;
  const double rpm = (state.velocity.x() / std::max(wheel_circumference, 1e-6)) * 60.0;
  wheelSpeeds.lf_speed = wheelSpeeds.rf_speed = wheelSpeeds.lb_speed = wheelSpeeds.rb_speed = static_cast<float>(rpm);
  wheelSpeeds.steering = static_cast<float>(input.delta);
  return wheelSpeeds;
}

double DynamicBicycle::calculateMagnitude(double x, double y) { return std::hypot(x, y); }

// -----------------------------------------------------------------------------
// Dynamic bicycle implementation

DynamicBicycle::Forces DynamicBicycle::computeForces(const VehicleState& x,
                                                     const VehicleInput& u,
                                                     double dt) const {
  if (!systems_configured_) {
    param_.validate();
    pt_.configure(param_.powertrain, static_cast<float>(param_.tire.radius));
    br_ = BrakeController(param_.brakes);
    systems_configured_ = true;
  }

  dt = std::max(dt, 1e-4);

  const double tau_th = std::max(param_.physics.tau_throttle, 1e-4);
  const double tau_br = std::max(param_.physics.tau_brake, 1e-4);
  const double a_th = std::exp(-dt / tau_th);
  const double a_br = std::exp(-dt / tau_br);

  const double thr_cmd = std::clamp(u.acc, 0.0, 1.0);
  const double brk_cmd = std::clamp(-u.acc, 0.0, 1.0);
  thr_eff_ = a_th * thr_eff_ + (1.0 - a_th) * thr_cmd;
  brk_eff_ = a_br * brk_eff_ + (1.0 - a_br) * brk_cmd;

  const double vx = x.velocity.x();
  const double vy = x.velocity.y();
  const double r = x.rotation.z();
  const auto low_ctx = evaluateLowSpeedContext(vx, vy);

  const double blend = low_ctx.blend;

  double alpha_front_target = u.delta - std::atan2(vy + param_.kinematic.l_F * r, low_ctx.vx_effective);
  double alpha_rear_target = std::atan2(vy - param_.kinematic.l_R * r, low_ctx.vx_effective);

  alpha_front_target *= blend;
  alpha_rear_target *= blend;

  const double relaxation_length = std::max(1.0, param_.tire.radius * 20.0);
  const double speed_for_relax = std::max(low_ctx.speed, 0.5);
  const double relax_rate = speed_for_relax / relaxation_length;
  const double lerp = 1.0 - std::exp(-relax_rate * dt);
  alpha_front_rel_ += (alpha_front_target - alpha_front_rel_) * lerp;
  alpha_rear_rel_ += (alpha_rear_target - alpha_rear_rel_) * lerp;

  if (low_ctx.speed < 0.35) {
    const double decay = std::exp(-(4.0 + 6.0 * brk_eff_) * dt);
    alpha_front_rel_ *= decay;
    alpha_rear_rel_ *= decay;
    if (low_ctx.speed < 0.05) {
      alpha_front_rel_ = 0.0;
      alpha_rear_rel_ = 0.0;
    }
  }

  const double alpha_f = std::clamp(alpha_front_rel_, -kHalfPi, kHalfPi);
  const double alpha_r = std::clamp(alpha_rear_rel_, -kHalfPi, kHalfPi);

  const double speed = low_ctx.speed;
  const double vx_long = std::abs(vx);

  const double drive_scale = std::max(0.25, blend);
  const double brake_scale = blend * blend;

  const double m = param_.inertia.m;
  const double Fdown = param_.aero.c_down * vx_long * vx_long;
  const double Fdrag = param_.aero.c_drag * vx * vx_long;
  const double rr_total = rollingResistance(param_.physics.c_rr, m * param_.inertia.g, vx);

  BrakeRequest breq = br_.prepare(static_cast<float>(brk_eff_), static_cast<float>(speed));
  PowertrainStatus ps = pt_.compute(static_cast<float>(thr_eff_), breq.regen_command,
                                    static_cast<float>(vx), static_cast<float>(dt));
  BrakeStatus bst = br_.finalize(breq, ps.regen_force);
  last_pt_status_ = ps;
  last_brake_status_ = bst;

  const double mu = (param_.tire.tire_coefficient > 0.0)
                        ? param_.tire.tire_coefficient
                        : std::max(0.1, std::abs(param_.tire.D));

  WheelLoads loads = computeWheelLoads(param_, Fdown, x.acceleration.x(), x.acceleration.y());

  WheelForceState fl{}, fr{}, rl{}, rr{};
  double Fy_front = 0.0;
  double Fy_rear = 0.0;
  double Fx_sum = 0.0;
  double mu_usage = 0.0;

  for (int iter = 0; iter < 2; ++iter) {
    fl.Fz = loads.Fz_fl;
    fr.Fz = loads.Fz_fr;
    rl.Fz = loads.Fz_rl;
    rr.Fz = loads.Fz_rr;

    const double Fy_front_coeff = pacejkaLat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double Fy_rear_coeff = pacejkaLat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

    fl.Fy = fl.Fz * Fy_front_coeff;
    fr.Fy = fr.Fz * Fy_front_coeff;
    rl.Fy = rl.Fz * Fy_rear_coeff;
    rr.Fy = rr.Fz * Fy_rear_coeff;

    fl.Fx = 0.5 * ps.front_drive_force * drive_scale;
    fr.Fx = 0.5 * ps.front_drive_force * drive_scale;
    rl.Fx = 0.5 * ps.rear_drive_force * drive_scale;
    rr.Fx = 0.5 * ps.rear_drive_force * drive_scale;

    fl.Fx -= 0.5 * (ps.front_regen_force + bst.front_force) * brake_scale;
    fr.Fx -= 0.5 * (ps.front_regen_force + bst.front_force) * brake_scale;
    rl.Fx -= 0.5 * (ps.rear_regen_force + bst.rear_force) * brake_scale;
    rr.Fx -= 0.5 * (ps.rear_regen_force + bst.rear_force) * brake_scale;

    if (std::abs(rr_total) > 1e-6 && loads.total() > 1e-6) {
      const double frac_fl = loads.Fz_fl / loads.total();
      const double frac_fr = loads.Fz_fr / loads.total();
      const double frac_rl = loads.Fz_rl / loads.total();
      const double frac_rr = loads.Fz_rr / loads.total();
      fl.Fx -= rr_total * frac_fl;
      fr.Fx -= rr_total * frac_fr;
      rl.Fx -= rr_total * frac_rl;
      rr.Fx -= rr_total * frac_rr;
    }

    double usage_fl = 0.0, usage_fr = 0.0, usage_rl = 0.0, usage_rr = 0.0;
    const auto clipped_fl = frictionCircleClip(fl.Fx, fl.Fy, mu * fl.Fz, usage_fl);
    const auto clipped_fr = frictionCircleClip(fr.Fx, fr.Fy, mu * fr.Fz, usage_fr);
    const auto clipped_rl = frictionCircleClip(rl.Fx, rl.Fy, mu * rl.Fz, usage_rl);
    const auto clipped_rr = frictionCircleClip(rr.Fx, rr.Fy, mu * rr.Fz, usage_rr);

    fl.Fx = clipped_fl.first;
    fl.Fy = clipped_fl.second;
    fr.Fx = clipped_fr.first;
    fr.Fy = clipped_fr.second;
    rl.Fx = clipped_rl.first;
    rl.Fy = clipped_rl.second;
    rr.Fx = clipped_rr.first;
    rr.Fy = clipped_rr.second;

    mu_usage = std::max({usage_fl, usage_fr, usage_rl, usage_rr});

    Fy_front = fl.Fy + fr.Fy;
    Fy_rear = rl.Fy + rr.Fy;
    Fx_sum = fl.Fx + fr.Fx + rl.Fx + rr.Fx;

    const double ax_raw = (r * vy + (Fx_sum - std::sin(u.delta) * Fy_front - Fdrag)) / m;
    const double ay_raw = ((std::cos(u.delta) * Fy_front) + Fy_rear) / m - r * vx;
    const double ax = std::clamp(ax_raw, -50.0, 50.0);
    const double ay = std::clamp(ay_raw, -50.0, 50.0);
    loads = computeWheelLoads(param_, Fdown, ax, ay);
  }

  Forces out{};
  out.Fx = Fx_sum - Fdrag;
  out.FyF = Fy_front;
  out.FyR = Fy_rear;

  last_debug_.alpha_front = alpha_f;
  last_debug_.alpha_rear = alpha_r;
  last_debug_.fy_front = Fy_front;
  last_debug_.fy_rear = Fy_rear;
  last_debug_.fx_total = out.Fx;
  last_debug_.mu_lat = std::min(mu_usage, 1.0);
  last_debug_.speed = speed;
  last_debug_.steer = u.delta;

  return out;
}

static VehicleState computeDerivatives(const VehicleParam& P,
                                       const VehicleState& x,
                                       const VehicleInput& u,
                                       const DynamicBicycle::Forces& F) {
  VehicleState xDot;
  const double m = P.inertia.m;
  const double Iz = P.inertia.I_z;
  const double lf = P.kinematic.l_F;
  const double lr = P.kinematic.l_R;

  const double s = std::sin(x.yaw);
  const double c = std::cos(x.yaw);

  xDot.position.x() = c * x.velocity.x() - s * x.velocity.y();
  xDot.position.y() = s * x.velocity.x() + c * x.velocity.y();
  xDot.yaw = x.rotation.z();

  xDot.velocity.x() = (x.rotation.z() * x.velocity.y() + (F.Fx - std::sin(u.delta) * F.FyF)) / m;
  xDot.velocity.y() = ((std::cos(u.delta) * F.FyF) + F.FyR) / m - x.rotation.z() * x.velocity.x();

  xDot.rotation.z() = (std::cos(u.delta) * F.FyF * lf - F.FyR * lr) / std::max(Iz, 1e-3);

  return xDot;
}

static void applyLowSpeedStabilisation(VehicleState& state,
                                       double brake_level,
                                       double blend,
                                       double dt) {
  const double low_weight = 1.0 - blend;
  if (low_weight <= 1e-6) return;

  const double gain = lowSpeedDampingGain(blend, brake_level);
  const double decay = std::exp(-gain * dt);
  state.velocity.y() *= decay;
  state.rotation.z() *= decay;

  if (std::hypot(state.velocity.x(), state.velocity.y()) < 0.05 && brake_level > 0.2) {
    state.velocity.x() = 0.0;
    state.velocity.y() = 0.0;
    state.rotation.z() = 0.0;
  }
}

void DynamicBicycle::updateState(VehicleState& state,
                                 const VehicleInput& rawInput,
                                 double dt) {
  VehicleInput u = rawInput;
  validateInput(u);

  Forces forces = computeForces(state, u, dt);

  VehicleState xDot = computeDerivatives(param_, state, u, forces);
  VehicleState next = state + xDot * dt;

  const auto ctx = evaluateLowSpeedContext(next.velocity.x(), next.velocity.y());
  applyLowSpeedStabilisation(next, brk_eff_, ctx.blend, dt);

  state = next;
  state.acceleration.x() = xDot.velocity.x();
  state.acceleration.y() = xDot.velocity.y();
  state.acceleration.z() = 0.0;

  validateState(state);
}
