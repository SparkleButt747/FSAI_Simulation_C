#include "DynamicBicycle.hpp"
#include "SanityChecks.hpp"
#include <algorithm>
#include <cmath>

using fsai::sim::sanity::checkInput;
using fsai::sim::sanity::checkState;
using fsai::sim::sanity::checkForces;
using fsai::sim::sanity::checkTireDebug;
using fsai::sim::sanity::fail;

namespace {
constexpr double kPi     = 3.141592653589793;
constexpr double kHalfPi = 1.5707963267948966;

constexpr double kBlendStart          = 0.6;
constexpr double kBlendEnd            = 1.8;
constexpr double kMinLongitudinalMag  = 0.35;
constexpr double kRelaxationLength    = 5.0;
constexpr double kMinRelaxationRate   = 5.0;
constexpr double kLowSpeedLeakRate    = 6.0;
constexpr double kRollFadeSpeed       = 0.6;

inline double smoothBlend01(double t){ t=std::clamp(t,0.0,1.0); return t*t*(3.0-2.0*t); }
inline double lowSpeedBlend(double v){
  if(v<=kBlendStart) return 0.0;
  if(v>=kBlendEnd) return 1.0;
  return smoothBlend01((v-kBlendStart)/std::max(1e-6,kBlendEnd-kBlendStart));
}
inline double regularizedLongitudinal(double vx,double blend){
  const double min_mag=kMinLongitudinalMag*(0.5+0.5*blend);
  const double s=(vx>=0.0)?1.0:-1.0;
  return s*std::max(std::abs(vx),min_mag);
}
inline double relaxationLerp(double speed,double blend,double dt){
  const double v_eff=std::max(0.3,speed);
  const double natural=v_eff/std::max(1e-3,kRelaxationLength);
  const double forced=kMinRelaxationRate*(1.0-blend);
  const double rate=std::max(natural,forced);
  const double lerp=1.0-std::exp(-rate*dt);
  return std::clamp(lerp,0.0,1.0);
}
inline double lowSpeedSlipLeak(double blend,double brake,double dt){
  const double leak_rate=(1.0-blend)*(kLowSpeedLeakRate+4.0*std::clamp(brake,0.0,1.0));
  if(leak_rate<=0.0) return 0.0;
  const double leak=1.0-std::exp(-leak_rate*dt);
  return std::clamp(leak,0.0,1.0);
}
inline double rollingResistanceForce(double base_rr,double vx){
  if(base_rr<=0.0) return 0.0;
  const double mag=std::abs(vx);
  if(mag<1e-6) return 0.0;
  const double s=(vx>=0.0)?1.0:-1.0;
  const double fade=std::tanh(mag/kRollFadeSpeed);
  return base_rr*fade*s;
}

struct WheelLoads { double Fz_fl,Fz_fr,Fz_rl,Fz_rr; };

WheelLoads computeWheelLoads(const VehicleParam& P,double Fdown,double ax,double ay){
  const double m=P.inertia.m, g=P.inertia.g;
  const double Wtot=m*g+Fdown, Wf=Wtot*P.kinematic.w_front, Wr=Wtot-Wf;
  double Fz_fl=0.5*Wf, Fz_fr=0.5*Wf, Fz_rl=0.5*Wr, Fz_rr=0.5*Wr;

  const double lf=P.kinematic.l_F, lr=P.kinematic.l_R, L=lf+lr;
  const double h=(P.physics.h_cg>0.0)?P.physics.h_cg:0.25;
  const double tw=std::max(0.3,P.kinematic.axle_width);

  const double dF_long=(m*ax*h)/std::max(1e-6,L);
  Fz_fl-=0.5*dF_long; Fz_fr-=0.5*dF_long; Fz_rl+=0.5*dF_long; Fz_rr+=0.5*dF_long;

  const double dF_lat=(m*ay*h)/tw;
  Fz_fl+=0.5*dF_lat; Fz_rl+=0.5*dF_lat; Fz_fr-=0.5*dF_lat; Fz_rr-=0.5*dF_lat;

  Fz_fl=std::max(0.0,Fz_fl); Fz_fr=std::max(0.0,Fz_fr); Fz_rl=std::max(0.0,Fz_rl); Fz_rr=std::max(0.0,Fz_rr);
  return {Fz_fl,Fz_fr,Fz_rl,Fz_rr};
}

inline double mf_lat(double slip,double B,double C,double D,double E){
  return D * std::sin(C * std::atan(B*(1.0-E)*slip + E*std::atan(B*slip)));
}

inline std::pair<double,double> frictionCircleClip(double Fx,double Fy,double muFz){
  muFz=std::max(1e-6,muFz); const double nx=Fx/muFz, ny=Fy/muFz;
  const double r=std::sqrt(nx*nx+ny*ny); if(r<=1.0) return {Fx,Fy};
  const double s=1.0/r; return {Fx*s,Fy*s};
}

} // namespace

DynamicBicycle::Forces DynamicBicycle::computeForces(const VehicleState& x,
                                                     const VehicleInput& u_in,
                                                     double dt) const {
  checkState(param_, x, "DynamicBicycle::computeForces(state)");
  checkInput(param_, u_in, "DynamicBicycle::computeForces(input)");
  if (!(dt>0.0 && std::isfinite(dt) && dt<1.0)) fail("DynamicBicycle::computeForces","dt invalid",dt);

  if (!systems_configured_) {
    pt_.configure(param_.powertrain, param_.tire.radius);
    br_ = BrakeController(param_.brakes);
    systems_configured_ = true;
  }

  // Actuator lags
  const double a_th = std::exp(-dt / std::max(1e-6, param_.physics.tau_throttle));
  const double a_br = std::exp(-dt / std::max(1e-6, param_.physics.tau_brake));
  const double thr_cmd = std::max(0.0, u_in.acc);
  const double brk_cmd = std::max(0.0, -u_in.acc);
  thr_eff_ = a_th * thr_eff_ + (1.0 - a_th) * thr_cmd;
  brk_eff_ = a_br * brk_eff_ + (1.0 - a_br) * brk_cmd;

  // Kinematics
  const double vx_body = x.velocity.x();
  const double vy      = x.velocity.y();
  const double r       = x.rotation.z();
  const double speed   = std::sqrt(vx_body*vx_body + vy*vy);
  const double blend   = lowSpeedBlend(speed);
  const double vx_mag  = std::abs(vx_body);
  const double vx      = vx_mag;

  // Aero / RR
  const double Fdown   = param_.aero.c_down * vx * vx;
  const double Fdrag   = param_.aero.c_drag * vx_body * vx;
  const double base_rr = param_.physics.c_rr * param_.inertia.m * param_.inertia.g;
  const double Frr     = rollingResistanceForce(base_rr, vx_body);

  // Slip (with relaxation)
  // Speed, friction
  const double speed_for_check_fric = std::hypot(x.velocity.x(), x.velocity.y());
  const double mu    = (param_.tire.tire_coefficient > 0.0)
                    ? param_.tire.tire_coefficient
                    : std::abs(param_.tire.D);
  const double r_max = rMaxFromFriction(speed_for_check_fric, mu, param_.inertia.g);

  // Steer soft-limit near yaw limit
  VehicleInput u_eff = u_in;
  u_eff.delta = steerSoftLimit(u_in.delta, x.rotation.z(), r_max);

  // Slip targets with limited steer
  const double alpha_f_target = getSlipAngle(x, u_eff, true);
  const double alpha_r_target = getSlipAngle(x, u_eff, false);


  // Gate slip sanity at low speed; slip isn't meaningful near standstill
  const double speed_for_check = std::hypot(x.velocity.x(), x.velocity.y());
  fsai::sim::sanity::checkSlipPair(alpha_f_target, alpha_r_target,
                                  "DynamicBicycle::slip_target",
                                  speed_for_check);

  const double relax = relaxationLerp(speed, blend, dt);
  alpha_front_rel_ += (alpha_f_target - alpha_front_rel_) * relax;
  alpha_rear_rel_  += (alpha_r_target - alpha_rear_rel_) * relax;

  const double leak = lowSpeedSlipLeak(blend, brk_eff_, dt);
  if (leak > 0.0) { alpha_front_rel_ *= (1.0 - leak); alpha_rear_rel_  *= (1.0 - leak); }

  if (speed < 0.25) {
    double decay_rate = 4.0 + 8.0 * std::max(0.0, brk_eff_);
    if (speed < 0.1) decay_rate += 6.0;
    const double decay = std::exp(-decay_rate * dt);
    alpha_front_rel_ *= decay; alpha_rear_rel_ *= decay;
    if ((speed < 0.05) || (speed < 0.15 && brk_eff_ > 0.05)) { alpha_front_rel_=0.0; alpha_rear_rel_=0.0; }
  }

  const double alpha_f = std::clamp(alpha_front_rel_, -kHalfPi, kHalfPi);
  const double alpha_r = std::clamp(alpha_rear_rel_,  -kHalfPi, kHalfPi);

  // Static axle loads and lateral
  const double m    = param_.inertia.m;
  const double Wtot = m * param_.inertia.g + Fdown;
  const double Wf   = Wtot * param_.kinematic.w_front;
  const double Wr   = Wtot - Wf;
  const double FyF_0 = Wf * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
  const double FyR_0 = Wr * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

  // Powertrain + brakes
  const BrakeRequest breq = br_.prepare(brk_eff_, vx);
  const PowertrainStatus ps = pt_.compute(thr_eff_, breq.regen_command, vx, dt);
  const BrakeStatus bst = br_.finalize(breq, ps.regen_force);
  last_pt_status_ = ps; last_brake_status_ = bst;

  fsai::sim::sanity::checkPowertrain(last_pt_status_, "DynamicBicycle::powertrain");
  fsai::sim::sanity::checkBrakes(last_brake_status_,  "DynamicBicycle::brakes");


  // Spin-aware longitudinal scaling: ease off drive/mech brake near yaw limit
  const double spin_scale = longForceSpinScale(x.rotation.z(), r_max);

  // Raw longitudinal (already included ps + brakes + drags)
  double Fx_drive      = ps.wheel_force * spin_scale;
  double Fx_mech_brake = bst.total_mechanical_force * spin_scale;

  if (brk_eff_ > 1e-3) {
    const double s = blend * blend;
    Fx_drive      *= s;
    Fx_mech_brake *= s;
  }
  double Fx_raw = Fx_drive - Fx_mech_brake - Fdrag - Frr;

  // Estimated accelerations for dynamic load transfer
  const double ax_est = (r * vy + (Fx_raw - std::sin(u_in.delta) * FyF_0)) / m;
  const double ay_est = ((std::cos(u_in.delta) * FyF_0) + FyR_0) / m - (r * vx_body);

  const auto WL = computeWheelLoads(param_, Fdown, ax_est, ay_est);

  // Per-wheel lateral (pre-circle)
  double Fy_fl = WL.Fz_fl * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
  double Fy_fr = WL.Fz_fr * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
  double Fy_rl = WL.Fz_rl * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
  double Fy_rr = WL.Fz_rr * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

  // Per-wheel longitudinal build
  const double torque_scale = (brk_eff_ > 1e-3) ? (blend * blend) : 1.0;

  const double Fd_front_each = 0.5 * ps.front_drive_force * torque_scale;
  const double Fd_rear_each  = 0.5 * ps.rear_drive_force  * torque_scale;
  const double Fr_front_each = 0.5 * ps.front_regen_force * torque_scale;
  const double Fr_rear_each  = 0.5 * ps.rear_regen_force  * torque_scale;

  const double Fb_front_each = 0.5 * bst.front_force * torque_scale;
  const double Fb_rear_each  = 0.5 * bst.rear_force  * torque_scale;

  double Fx_fl = +Fd_front_each - Fr_front_each - Fb_front_each;
  double Fx_fr = +Fd_front_each - Fr_front_each - Fb_front_each;
  double Fx_rl = +Fd_rear_each  - Fr_rear_each  - Fb_rear_each;
  double Fx_rr = +Fd_rear_each  - Fr_rear_each  - Fb_rear_each;

  const double totalFz = WL.Fz_fl + WL.Fz_fr + WL.Fz_rl + WL.Fz_rr;
  if (std::abs(Frr) > 1e-6 && totalFz > 1e-6) {
    Fx_fl -= Frr * (WL.Fz_fl / totalFz);
    Fx_fr -= Frr * (WL.Fz_fr / totalFz);
    Fx_rl -= Frr * (WL.Fz_rl / totalFz);
    Fx_rr -= Frr * (WL.Fz_rr / totalFz);
  }

  // Friction circle
  auto cfl = frictionCircleClip(Fx_fl, Fy_fl, mu * WL.Fz_fl);
  auto cfr = frictionCircleClip(Fx_fr, Fy_fr, mu * WL.Fz_fr);
  auto crl = frictionCircleClip(Fx_rl, Fy_rl, mu * WL.Fz_rl);
  auto crr = frictionCircleClip(Fx_rr, Fy_rr, mu * WL.Fz_rr);

  Fx_fl = cfl.first; Fy_fl = cfl.second;
  Fx_fr = cfr.first; Fy_fr = cfr.second;
  Fx_rl = crl.first; Fy_rl = crl.second;
  Fx_rr = crr.first; Fy_rr = crr.second;

  Forces out{};
  out.Fx  = Fx_fl + Fx_fr + Fx_rl + Fx_rr;
  out.FyF = Fy_fl + Fy_fr;
  out.FyR = Fy_rl + Fy_rr;

  // Debug payload (for sanity checks too)
  {
    const double Fz_sum = WL.Fz_fl + WL.Fz_fr + WL.Fz_rl + WL.Fz_rr;
    double mu_lat = 0.0; if (Fz_sum > 1e-6) mu_lat = (std::abs(out.FyF) + std::abs(out.FyR)) / Fz_sum;
    tire_debug_.alpha_front = alpha_f;
    tire_debug_.alpha_rear  = alpha_r;
    tire_debug_.fy_front    = out.FyF;
    tire_debug_.fy_rear     = out.FyR;
    tire_debug_.fx_total    = out.Fx;
    tire_debug_.mu_lat      = mu_lat;
    tire_debug_.speed       = speed;
    tire_debug_.steer       = u_in.delta;
  }

  checkForces(param_, out.Fx, out.FyF, out.FyR, "DynamicBicycle::computeForces(out)");
  checkTireDebug(param_, tire_debug_.alpha_front, tire_debug_.alpha_rear,
                 tire_debug_.fy_front, tire_debug_.fy_rear, tire_debug_.fx_total,
                 tire_debug_.mu_lat, tire_debug_.speed, tire_debug_.steer,
                 "DynamicBicycle::computeForces(tire_debug)");

  return out;
}

// Simple RHS for integration (with damping terms)
static VehicleState fDynamics(const DynamicBicycle& db,
                              const VehicleState& x,
                              const VehicleInput& u,
                              double Fx, double FyF, double FyR,
                              double low_weight, double brake_level) {
  VehicleState xDot;
  const double m  = db.param().inertia.m;
  const double Iz = db.param().inertia.I_z;
  const double lf = db.param().kinematic.l_F;
  const double lr = db.param().kinematic.l_R;

  const double s = std::sin(x.yaw), c = std::cos(x.yaw);

  xDot.position.x() = c * x.velocity.x() - s * x.velocity.y();
  xDot.position.y() = s * x.velocity.x() + c * x.velocity.y();
  xDot.yaw          = x.rotation.z();

  xDot.velocity.x() = (x.rotation.z()*x.velocity.y() + (Fx - std::sin(u.delta)*FyF)) / m;
  xDot.velocity.y() = ((std::cos(u.delta)*FyF) + FyR) / m - (x.rotation.z()*x.velocity.x());

  // Yaw from tire moments
double rDot = (std::cos(u.delta) * FyF * lf - FyR * lr) / Iz;

// Add yaw governor to keep |r| <= r_max
const double g = db.param().inertia.g;
const double mu = std::max(0.4, db.param().tire.tire_coefficient); // floor
const double speed = std::hypot(x.velocity.x(), x.velocity.y());
const double r_max = db.rMaxFromFriction(speed, mu, g);
rDot += db.yawGovernor(x.rotation.z(), r_max, speed);

// Existing damping terms (they still help)
xDot.rotation.z() = rDot;


  // Damping
  xDot.velocity.y() -= 0.25 * x.velocity.y();
  xDot.rotation.z() -= 0.18 * x.rotation.z();

  if (low_weight > 1e-6) {
    const double brk = std::clamp(brake_level, 0.0, 1.0);
    const double lat_damp = (6.0 + 8.0 * brk) * low_weight;
    const double yaw_damp = (3.0 + 5.0 * brk) * low_weight;
    const double vx_damp  = (1.5 + 3.0 * brk) * low_weight;
    xDot.velocity.x() -= vx_damp  * x.velocity.x();
    xDot.velocity.y() -= lat_damp * x.velocity.y();
    xDot.rotation.z() -= yaw_damp * x.rotation.z();
  }

  return xDot;
}

void DynamicBicycle::updateState(VehicleState& state,
                                 const VehicleInput& inputRaw,
                                 double dt) {
  checkState(param_, state, "DynamicBicycle::updateState(pre-state)");
  checkInput(param_, inputRaw, "DynamicBicycle::updateState(input)");
  if (!(dt>0.0 && std::isfinite(dt) && dt<1.0)) fail("DynamicBicycle::updateState","dt invalid",dt);

  VehicleInput u = inputRaw;
  validateInput(u);
  fsai::sim::sanity::checkFullStepPre(param_, state, u, "DynamicBicycle::pre");

  const Forces F = computeForces(state, u, dt);

  fsai::sim::sanity::ForceSummary Fsum{F.Fx, F.FyF, F.FyR};
  fsai::sim::sanity::checkForces(Fsum, "DynamicBicycle::forces");


  const double speed = std::sqrt(state.velocity.x()*state.velocity.x() + state.velocity.y()*state.velocity.y());
  const double blend = (speed<=kBlendStart)?0.0:((speed>=kBlendEnd)?1.0:smoothBlend01((speed-kBlendStart)/std::max(1e-6,kBlendEnd-kBlendStart)));
  const double low_weight = 1.0 - blend;

  VehicleState xDot  = fDynamics(*this, state, u, F.Fx, F.FyF, F.FyR, low_weight, brk_eff_);

  fsai::sim::sanity::checkDerivatives(xDot, "DynamicBicycle::xDot");

  VehicleState xNext = state + xDot * dt;

  if (fsai::sim::sanity::Limits{}.wrap_yaw_to_pi) {
    fsai::sim::sanity::wrapYaw(xNext);
  }
  fsai::sim::sanity::checkState(xNext, "DynamicBicycle::xNext");


  const double speed_next = calculateMagnitude(xNext.velocity.x(), xNext.velocity.y());
  if (speed_next < 0.3) {
    const double settle_brake = std::clamp(brk_eff_, 0.0, 1.0);
    const double lat_tau = 0.12 + 0.25 * (1.0 - settle_brake);
    const double yaw_tau = 0.15 + 0.35 * (1.0 - settle_brake);
    const double vx_tau  = 0.20 + 0.30 * (1.0 - settle_brake);
    const double lat_decay = std::exp(-dt / lat_tau);
    const double yaw_decay = std::exp(-dt / yaw_tau);
    const double vx_decay  = std::exp(-dt / vx_tau);
    xNext.velocity.y() *= lat_decay;
    xNext.rotation.z() *= yaw_decay;
    xNext.velocity.x() *= vx_decay;

    if (speed_next < 0.05 && settle_brake > 0.2) {
      xNext.velocity.x() = 0.0; xNext.velocity.y() = 0.0; xNext.rotation.z() = 0.0;
    }
  }

  state = xNext;
  state.acceleration.x() = xDot.velocity.x();
  state.acceleration.y() = xDot.velocity.y();
  state.acceleration.z() = 0.0;

  fsai::sim::sanity::ForceSummary Fsum2{F.Fx, F.FyF, F.FyR};
  auto wi2 = getWheelSpeeds(state, u);
  fsai::sim::sanity::checkFullStepPost(state, xDot, Fsum2, lastPowertrainStatus(),
                                     lastBrakeStatus(), wi2, "DynamicBicycle::post");


  checkState(param_, state, "DynamicBicycle::updateState(post-state)");
}
