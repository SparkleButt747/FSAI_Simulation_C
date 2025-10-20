#include "DynamicBicycle.hpp"
#include <yaml.h>
#include <stdexcept>
#include <cmath>
#include <algorithm>

using std::cos;
using std::sin;
using std::atan2;
using std::tan;

namespace {
inline double sgn(double v){ return (v>0)-(v<0); }

struct WheelLoads { double Fz_fl, Fz_fr, Fz_rl, Fz_rr; };

static WheelLoads computeWheelLoads(const VehicleParam& P, double Fdown,
                                    double ax, double ay) {
    // static + aero split by w_front
    const double m = P.inertia.m;
    const double g = P.inertia.g;
    const double Wtot = m*g + Fdown;

    const double Wf = Wtot * P.kinematic.w_front;
    const double Wr = Wtot - Wf;

    // start from static per-wheel
    double Fz_fl = 0.5 * Wf, Fz_fr = 0.5 * Wf;
    double Fz_rl = 0.5 * Wr, Fz_rr = 0.5 * Wr;

    // dynamic transfer
    const double lf = P.kinematic.l_F;
    const double lr = P.kinematic.l_R;
    const double L  = lf + lr;
    const double h  = (P.physics.h_cg > 0.0)? P.physics.h_cg : 0.25;
    const double tw = std::max(0.3, P.kinematic.axle_width);

    // longitudinal: front loses under accel
    const double dF_long = (m * ax * h) / std::max(1e-6, L);
    Fz_fl -= 0.5 * dF_long; Fz_fr -= 0.5 * dF_long;
    Fz_rl += 0.5 * dF_long; Fz_rr += 0.5 * dF_long;

    // lateral: ay>0 -> load to left side
    const double dF_lat = (m * ay * h) / tw;
    Fz_fl += 0.5 * dF_lat; Fz_rl += 0.5 * dF_lat;
    Fz_fr -= 0.5 * dF_lat; Fz_rr -= 0.5 * dF_lat;

    // non-negative
    Fz_fl = std::max(0.0, Fz_fl); Fz_fr = std::max(0.0, Fz_fr);
    Fz_rl = std::max(0.0, Fz_rl); Fz_rr = std::max(0.0, Fz_rr);
    return {Fz_fl, Fz_fr, Fz_rl, Fz_rr};
}

// Pacejka-like lateral shape (unitless) -> multiply by Fz
static inline double mf_lat(double slip, double B, double C, double D, double E){
    return D * sin(C * atan(B*(1.0 - E)*slip + E * atan(B*slip)));
}

// Ellipse clipping helper, better than capping by μFz
static inline std::pair<double,double> ellipse_clip(double Fx, double Fy, double muFz){
    muFz = std::max(1e-6, muFz);
    const double nx = Fx / muFz, ny = Fy / muFz;
    const double r = std::sqrt(nx*nx + ny*ny);
    if (r <= 1.0) return {Fx, Fy};
    const double s = 1.0 / r;
    return {Fx*s, Fy*s};
}
}

// ---------------- VehicleModel (unchanged bits) ------------------

void VehicleModel::updateState(VehicleState&, const VehicleInput&, double) {
    // Base class does nothing
}
void VehicleModel::validateState(VehicleState& state) const { state.velocity.x() = std::max(0.0, state.velocity.x()); }

void VehicleModel::validateInput(VehicleInput& input) const {
    input.acc   = std::clamp(input.acc,   param_.input_ranges.acc.min,   param_.input_ranges.acc.max);
    input.vel   = std::clamp(input.vel,   param_.input_ranges.vel.min,   param_.input_ranges.vel.max);
    input.delta = std::clamp(input.delta, param_.input_ranges.delta.min, param_.input_ranges.delta.max);
}

double VehicleModel::getSlipAngle(const VehicleState& x, const VehicleInput& u, bool isFront) const {
    const double vx = std::max(0.1, x.velocity.x());
    const double vy = x.velocity.y();
    const double r  = x.rotation.z();
    const double lf = param_.kinematic.l_F;
    const double lr = param_.kinematic.l_R;
    if (isFront) return atan2(vy + lf * r, vx) - u.delta;
    return atan2(vy - lr * r, vx);
}
WheelsInfo VehicleModel::getWheelSpeeds(const VehicleState& state, const VehicleInput& input) const {
    WheelsInfo wheelSpeeds = WheelsInfo_default();
    double wheelCircumference = 2.0 * M_PI * param_.tire.radius;
    double rpm = (state.velocity.x() / std::max(1e-12, wheelCircumference)) * 60.0;
    wheelSpeeds.lf_speed = wheelSpeeds.rf_speed = wheelSpeeds.lb_speed = wheelSpeeds.rb_speed = static_cast<float>(rpm);
    wheelSpeeds.steering = static_cast<float>(input.delta);
    return wheelSpeeds;
}

double DynamicBicycle::calculateMagnitude(double x, double y){ return std::sqrt(x*x + y*y); }

// ---------------- DynamicBicycle ------------------

DynamicBicycle::Forces DynamicBicycle::computeForces(const VehicleState& x, const VehicleInput& u_in, double dt) const {
    // lazy init powertrain/brake with config
    if (pt_.soc() == 0.f && param_.powertrain.battery.soc_init > 0.f) {
        pt_.configure(param_.powertrain, static_cast<float>(param_.tire.radius));
        br_ = BrakeController(param_.brakes);
    }

    // Actuator lags
    const double tau_th = std::max(1e-6, param_.physics.tau_throttle);
    const double tau_br = std::max(1e-6, param_.physics.tau_brake);
    const double a_th = std::exp(-dt / tau_th);
    const double a_br = std::exp(-dt / tau_br);

    const double thr_cmd = std::max(0.0, u_in.acc);
    const double brk_cmd = std::max(0.0, -u_in.acc);

    thr_eff_ = a_th * thr_eff_ + (1.0 - a_th) * thr_cmd;
    brk_eff_ = a_br * brk_eff_ + (1.0 - a_br) * brk_cmd;

    const double vx = std::max(0.0, x.velocity.x());
    const double vy = x.velocity.y();
    const double r  = x.rotation.z();

    // Aero + rolling resistance
    const double Fdrag = param_.aero.c_drag * vx * vx;
    const double Fdown = param_.aero.c_down * vx * vx;
    const double Frr   = param_.physics.c_rr * param_.inertia.m * param_.inertia.g * sgn(vx);

    // First pass: static loads → lateral forces
    const double Wtot = param_.inertia.m * param_.inertia.g + Fdown;
    const double Wf   = Wtot * param_.kinematic.w_front;
    const double Wr   = Wtot - Wf;
    double perWheelF = 0.5 * Wf, perWheelR = 0.5 * Wr;

    const double alpha_f_0 = getSlipAngle(x, u_in, true);
    const double alpha_r_0 = getSlipAngle(x, u_in, false);
    const double FyF_0 = (perWheelF*2.0) * mf_lat(alpha_f_0, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double FyR_0 = (perWheelR*2.0) * mf_lat(alpha_r_0, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

    // Longitudinal from EV + brakes
    const BrakeRequest breq = br_.prepare(static_cast<float>(brk_eff_), static_cast<float>(vx));
    const PowertrainStatus ps = pt_.compute(static_cast<float>(thr_eff_), breq.regen_command, static_cast<float>(vx), static_cast<float>(dt));
    const BrakeStatus     bst = br_.finalize(breq, ps.regen_force);

    // Raw Fx components (before ellipse), sign convention: forward positive
    double Fx_drive = double(ps.wheel_force);
    double Fx_mech_brake = double(bst.total_mechanical_force);
    double Fx_raw = Fx_drive - Fdrag - Frr - Fx_mech_brake;

    // Estimate ax, ay (body) using first-pass Fy for dynamic load transfer
    const double m = param_.inertia.m;
    const double FyFtot0 = 2.0 * FyF_0, FyRtot0 = 2.0 * FyR_0;
    const double ax_est = (r * vy + (Fx_raw - std::sin(u_in.delta) * FyFtot0)) / m;
    const double ay_est = ((std::cos(u_in.delta) * FyFtot0) + FyRtot0) / m - (r * vx);

    // Dynamic loads
    WheelLoads WL = computeWheelLoads(param_, Fdown, ax_est, ay_est);
    perWheelF = 0.5*(WL.Fz_fl + WL.Fz_fr);
    perWheelR = 0.5*(WL.Fz_rl + WL.Fz_rr);

    // Recompute lateral with dynamic Fz
    const double alpha_f = alpha_f_0; // slip angles unchanged this dt
    const double alpha_r = alpha_r_0;
    const double FyF = (perWheelF*2.0) * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double FyR = (perWheelR*2.0) * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

    // Per-wheel split & ellipse
    const double mu = (param_.tire.tire_coefficient>0.0)? param_.tire.tire_coefficient : std::abs(param_.tire.D);

    // Lateral split equally on each axle
    double Fy_fl = 0.5 * FyF, Fy_fr = 0.5 * FyF;
    double Fy_rl = 0.5 * FyR, Fy_rr = 0.5 * FyR;

    // Longitudinal split: drive/regen by PT (front/rear) + mechanical brakes by bias
    double Fx_fl=0, Fx_fr=0, Fx_rl=0, Fx_rr=0;

    double Fd_front_each = 0.5 * ps.front_drive_force;
    double Fd_rear_each  = 0.5 * ps.rear_drive_force;
    double Fr_front_each = 0.5 * ps.front_regen_force;
    double Fr_rear_each  = 0.5 * ps.rear_regen_force;

    Fx_fl += Fd_front_each - Fr_front_each;
    Fx_fr += Fd_front_each - Fr_front_each;
    Fx_rl += Fd_rear_each  - Fr_rear_each;
    Fx_rr += Fd_rear_each  - Fr_rear_each;

    double Fb_front_each = 0.5 * bst.front_force;
    double Fb_rear_each  = 0.5 * bst.rear_force;
    Fx_fl -= Fb_front_each; Fx_fr -= Fb_front_each;
    Fx_rl -= Fb_rear_each;  Fx_rr -= Fb_rear_each;

    // Ellipse clipping with μFz caps (per wheel)
    auto clipped_fl = ellipse_clip(Fx_fl, Fy_fl, mu * WL.Fz_fl);
    auto clipped_fr = ellipse_clip(Fx_fr, Fy_fr, mu * WL.Fz_fr);
    auto clipped_rl = ellipse_clip(Fx_rl, Fy_rl, mu * WL.Fz_rl);
    auto clipped_rr = ellipse_clip(Fx_rr, Fy_rr, mu * WL.Fz_rr);

    Fx_fl = clipped_fl.first; Fy_fl = clipped_fl.second;
    Fx_fr = clipped_fr.first; Fy_fr = clipped_fr.second;
    Fx_rl = clipped_rl.first; Fy_rl = clipped_rl.second;
    Fx_rr = clipped_rr.first; Fy_rr = clipped_rr.second;

    // Sum totals (drag & rolling already accounted in Fx_raw path)
    Forces out{};
    out.Fx  = Fx_fl + Fx_fr + Fx_rl + Fx_rr - 0.0;
    out.FyF = Fy_fl + Fy_fr;
    out.FyR = Fy_rl + Fy_rr;
    return out;
}

static VehicleState fDynamics(const DynamicBicycle& db, const VehicleState& x, const VehicleInput& u,
                              double Fx, double FyF, double FyR){
    VehicleState xDot;
    const double m  = db.param().inertia.m;
    const double Iz = db.param().inertia.I_z;
    const double lf = db.param().kinematic.l_F;
    const double lr = db.param().kinematic.l_R;

    const double s   = std::sin(x.yaw);
    const double c   = std::cos(x.yaw);
    const double FyFtot = 2.0 * FyF;
    const double FyRtot = 2.0 * FyR;

    // World position kinematics
    xDot.position.x() = c * x.velocity.x() - s * x.velocity.y();
    xDot.position.y() = s * x.velocity.x() + c * x.velocity.y();
    xDot.yaw          = x.rotation.z();

    // Body accelerations
    xDot.velocity.x() = (x.rotation.z() * x.velocity.y() + (Fx - std::sin(u.delta) * FyFtot)) / m;
    xDot.velocity.y() = ((std::cos(u.delta) * FyFtot) + FyRtot) / m - (x.rotation.z() * x.velocity.x());

    // Yaw dynamics
    xDot.rotation.z() = (std::cos(u.delta) * FyFtot * lf - FyRtot * lr) / Iz;

    return xDot;
}

void DynamicBicycle::updateState(VehicleState& state, const VehicleInput& inputRaw, double dt) {
    VehicleInput u = inputRaw;
    validateInput(u);

    // compute forces with v2 physics
    Forces F = computeForces(state, u, dt);

    // integrate
    VehicleState xDot  = fDynamics(*this, state, u, F.Fx, F.FyF, F.FyR);
    VehicleState xNext = state + xDot * dt;

    // near-zero guard
    const double v = calculateMagnitude(xNext.velocity.x(), xNext.velocity.y());
    if (v < 0.05) { xNext.velocity.y() = 0.0; xNext.rotation.z() = 0.0; }

    state = xNext;
    state.acceleration.x() = xDot.velocity.x();
    state.acceleration.y() = xDot.velocity.y();
    state.acceleration.z() = 0.0;

    validateState(state);
}
