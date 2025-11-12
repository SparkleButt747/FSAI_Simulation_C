#include "DynamicBicycle.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

using std::cos;
using std::sin;
using std::atan2;
using std::tan;

namespace {
constexpr double kBlendStart          = 0.6;   // [m/s] begin reducing slip forces
constexpr double kBlendEnd            = 1.8;   // [m/s] full dynamic behaviour restored
constexpr double kHalfPi              = 1.5707963267948966;
constexpr double kMinLongitudinalMag  = 0.20;  // [m/s] prevents singular slip angles
constexpr double kRelaxationLength    = 5.0;   // [m] pneumatic trail approximation
constexpr double kMinRelaxationRate   = 5.0;   // [1/s] ensures slip states bleed off when nearly stopped
constexpr double kLowSpeedLeakRate    = 6.0;   // [1/s] extra decay for slip states at standstill
constexpr double kRollFadeSpeed       = 0.6;   // [m/s] fade rolling resistance near standstill

inline double smoothBlend01(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

inline double lowSpeedBlend(double speed) {
    if (speed <= kBlendStart) return 0.0;
    if (speed >= kBlendEnd)   return 1.0;
    const double span = std::max(1e-6, kBlendEnd - kBlendStart);
    return smoothBlend01((speed - kBlendStart) / span);
}

inline double regularizedLongitudinal(double vx, double blend) {
    const double min_mag = kMinLongitudinalMag * (0.5 + 0.5 * blend);
    const double sign    = (vx >= 0.0) ? 1.0 : -1.0;
    const double mag     = std::max(std::abs(vx), min_mag);
    return sign * mag;
}

inline double relaxationLerp(double speed, double blend, double dt) {
    const double v_eff = std::max(0.3, speed);
    const double natural_rate = v_eff / std::max(1e-3, kRelaxationLength);
    const double forced_rate  = kMinRelaxationRate * (1.0 - blend);
    const double rate         = std::max(natural_rate, forced_rate);
    const double lerp         = 1.0 - std::exp(-rate * dt);
    return std::clamp(lerp, 0.0, 1.0);
}

inline double lowSpeedSlipLeak(double blend, double brake, double dt) {
    const double leak_rate = (1.0 - blend) * (kLowSpeedLeakRate + 4.0 * std::clamp(brake, 0.0, 1.0));
    if (leak_rate <= 0.0) return 0.0;
    const double leak = 1.0 - std::exp(-leak_rate * dt);
    return std::clamp(leak, 0.0, 1.0);
}

inline double rollingResistanceForce(double base_rr, double vx) {
    if (base_rr <= 0.0) return 0.0;
    const double mag  = std::abs(vx);
    if (mag < 1e-6) return 0.0;
    const double sign = (vx >= 0.0) ? 1.0 : -1.0;
    const double fade = std::tanh(mag / kRollFadeSpeed);
    return base_rr * fade * sign;
}

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

// Combined-slip limit helper (actually a friction circle)
static inline std::pair<double,double> frictionCircleClip(double Fx, double Fy, double muFz){
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
void VehicleModel::validateState(VehicleState& state) const { (void)state; }

void VehicleModel::validateInput(VehicleInput& input) const {
    input.acc   = std::clamp(input.acc,   param_.input_ranges.acc.min,   param_.input_ranges.acc.max);
    input.vel   = std::clamp(input.vel,   param_.input_ranges.vel.min,   param_.input_ranges.vel.max);
    input.delta = std::clamp(input.delta, param_.input_ranges.delta.min, param_.input_ranges.delta.max);
}

double VehicleModel::getSlipAngle(const VehicleState& x, const VehicleInput& u, bool isFront) const {
    const double vx_raw = x.velocity.x();
    const double vy     = x.velocity.y();
    const double r      = x.rotation.z();
    const double lf     = param_.kinematic.l_F;
    const double lr     = param_.kinematic.l_R;

    const double speed  = std::sqrt(vx_raw * vx_raw + vy * vy);
    const double blend  = lowSpeedBlend(speed);
    const double vx_eff = regularizedLongitudinal(vx_raw, blend);

    double slip;
    if (isFront) {
        slip = atan2(vy + lf * r, vx_eff) - u.delta;
    } else {
        slip = atan2(vy - lr * r, vx_eff);
    }
    return std::clamp(slip, -kHalfPi, kHalfPi);
}
WheelsInfo VehicleModel::getWheelSpeeds(const VehicleState& state, const VehicleInput& input) const {
    WheelsInfo wheelSpeeds = WheelsInfo_default();
    double wheelCircumference = 2.0 * ::kPi * param_.tire.radius;
    double rpm = (state.velocity.x() / std::max(1e-12, wheelCircumference)) * 60.0;
    wheelSpeeds.lf_speed = wheelSpeeds.rf_speed = wheelSpeeds.lb_speed = wheelSpeeds.rb_speed = static_cast<float>(rpm);
    wheelSpeeds.steering = static_cast<float>(input.delta);
    return wheelSpeeds;
}

double DynamicBicycle::calculateMagnitude(double x, double y){ return std::sqrt(x*x + y*y); }

// ---------------- DynamicBicycle ------------------

DynamicBicycle::Forces DynamicBicycle::computeForces(const VehicleState& x, const VehicleInput& u_in, double dt) const {
    // lazy init powertrain/brake with config
    if (!systems_configured_) {
        param_.validate();
        pt_.configure(param_.powertrain, static_cast<float>(param_.tire.radius));
        br_ = BrakeController(param_.brakes);
        systems_configured_ = true;
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

    const double vx_body = x.velocity.x();
    const double vx_mag  = std::abs(vx_body);
    const double vx = vx_mag;
    const double vy = x.velocity.y();
    const double r  = x.rotation.z();
    const double speed = std::sqrt(vx_body * vx_body + vy * vy);
    const double slip_blend = lowSpeedBlend(speed);

    // Aero + rolling resistance
    const double Fdown = param_.aero.c_down * vx * vx;
    const double Fdrag = param_.aero.c_drag * vx_body * vx;
    const double base_rr = param_.physics.c_rr * param_.inertia.m * param_.inertia.g;
    const double Frr   = rollingResistanceForce(base_rr, vx_body);

    // First pass: static loads → lateral forces
    const double Wtot = param_.inertia.m * param_.inertia.g + Fdown;
    const double Wf   = Wtot * param_.kinematic.w_front;
    const double Wr   = Wtot - Wf;
    const double alpha_f_target = getSlipAngle(x, u_in, true);
    const double alpha_r_target = getSlipAngle(x, u_in, false);

    const double relax_lerp = relaxationLerp(speed, slip_blend, dt);
    alpha_front_rel_ += (alpha_f_target - alpha_front_rel_) * relax_lerp;
    alpha_rear_rel_  += (alpha_r_target - alpha_rear_rel_) * relax_lerp;

    const double leak = lowSpeedSlipLeak(slip_blend, brk_eff_, dt);
    if (leak > 0.0) {
        alpha_front_rel_ *= (1.0 - leak);
        alpha_rear_rel_  *= (1.0 - leak);
    }
    if (speed < 0.25) {
        const double base_rate   = 4.0;
        const double brake_bonus = 6.0 * std::clamp(brk_eff_, 0.0, 1.0);
        const double decay_rate  = base_rate + brake_bonus;
        const double decay       = std::exp(-decay_rate * dt);
        alpha_front_rel_ *= decay;
        alpha_rear_rel_  *= decay;
        if (speed < 0.05) {
            alpha_front_rel_ = 0.0;
            alpha_rear_rel_  = 0.0;
        }
    }

    const double alpha_f = std::clamp(alpha_front_rel_, -kHalfPi, kHalfPi);
    const double alpha_r = std::clamp(alpha_rear_rel_,  -kHalfPi, kHalfPi);

    const double FyF_0 = Wf * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double FyR_0 = Wr * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

    // Longitudinal from EV + brakes
    const BrakeRequest breq = br_.prepare(static_cast<float>(brk_eff_), static_cast<float>(vx));
    const PowertrainStatus ps = pt_.compute(static_cast<float>(thr_eff_), breq.regen_command, static_cast<float>(vx), static_cast<float>(dt));
    const BrakeStatus     bst = br_.finalize(breq, ps.regen_force);
    last_pt_status_ = ps;
    last_brake_status_ = bst;

    // Raw Fx components (before ellipse), sign convention: forward positive
    const double torque_scale = (brk_eff_ > 1e-3) ? (slip_blend * slip_blend) : 1.0;
    const double Fx_drive_total = double(ps.wheel_force);
    const double Fx_mech_brake_total = double(bst.total_mechanical_force);
    const double Fx_raw = Fx_drive_total * torque_scale - Fdrag - Frr - Fx_mech_brake_total * torque_scale;

    // Estimate ax, ay (body) using first-pass Fy for dynamic load transfer
    const double m = param_.inertia.m;
    const double FyFtot0 = FyF_0;
    const double FyRtot0 = FyR_0;
    const double ax_est = (r * vy + (Fx_raw - std::sin(u_in.delta) * FyFtot0)) / m;
    const double ay_est = ((std::cos(u_in.delta) * FyFtot0) + FyRtot0) / m - (r * vx_body);

    // Dynamic loads
    WheelLoads WL = computeWheelLoads(param_, Fdown, ax_est, ay_est);
    // Recompute lateral with dynamic Fz
    const double Fy_fl_stat = WL.Fz_fl * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double Fy_fr_stat = WL.Fz_fr * mf_lat(alpha_f, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double Fy_rl_stat = WL.Fz_rl * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);
    const double Fy_rr_stat = WL.Fz_rr * mf_lat(alpha_r, param_.tire.B, param_.tire.C, param_.tire.D, param_.tire.E);

    double Fy_fl = Fy_fl_stat;
    double Fy_fr = Fy_fr_stat;
    double Fy_rl = Fy_rl_stat;
    double Fy_rr = Fy_rr_stat;

    // Per-wheel split & ellipse
    const double mu = (param_.tire.tire_coefficient>0.0)? param_.tire.tire_coefficient : std::abs(param_.tire.D);

    // Longitudinal split: drive/regen by PT (front/rear) + mechanical brakes by bias
    double Fx_fl=0, Fx_fr=0, Fx_rl=0, Fx_rr=0;

    double Fd_front_each = 0.5 * ps.front_drive_force * torque_scale;
    double Fd_rear_each  = 0.5 * ps.rear_drive_force  * torque_scale;
    double Fr_front_each = 0.5 * ps.front_regen_force * torque_scale;
    double Fr_rear_each  = 0.5 * ps.rear_regen_force  * torque_scale;

    Fx_fl += Fd_front_each - Fr_front_each;
    Fx_fr += Fd_front_each - Fr_front_each;
    Fx_rl += Fd_rear_each  - Fr_rear_each;
    Fx_rr += Fd_rear_each  - Fr_rear_each;

    double Fb_front_each = 0.5 * bst.front_force * torque_scale;
    double Fb_rear_each  = 0.5 * bst.rear_force  * torque_scale;
    Fx_fl -= Fb_front_each; Fx_fr -= Fb_front_each;
    Fx_rl -= Fb_rear_each;  Fx_rr -= Fb_rear_each;

    // distribute rolling resistance into the per-wheel longitudinal channels so it competes with available grip
    const double total_Fz = WL.Fz_fl + WL.Fz_fr + WL.Fz_rl + WL.Fz_rr;
    if (std::abs(Frr) > 1e-6 && total_Fz > 1e-6) {
        const double rr_fl = Frr * (WL.Fz_fl / total_Fz);
        const double rr_fr = Frr * (WL.Fz_fr / total_Fz);
        const double rr_rl = Frr * (WL.Fz_rl / total_Fz);
        const double rr_rr = Frr * (WL.Fz_rr / total_Fz);
        Fx_fl -= rr_fl;
        Fx_fr -= rr_fr;
        Fx_rl -= rr_rl;
        Fx_rr -= rr_rr;
    }

    // Combined-slip clipping with μFz caps (per wheel)
    auto clipped_fl = frictionCircleClip(Fx_fl, Fy_fl, mu * WL.Fz_fl);
    auto clipped_fr = frictionCircleClip(Fx_fr, Fy_fr, mu * WL.Fz_fr);
    auto clipped_rl = frictionCircleClip(Fx_rl, Fy_rl, mu * WL.Fz_rl);
    auto clipped_rr = frictionCircleClip(Fx_rr, Fy_rr, mu * WL.Fz_rr);

    Fx_fl = clipped_fl.first; Fy_fl = clipped_fl.second;
    Fx_fr = clipped_fr.first; Fy_fr = clipped_fr.second;
    Fx_rl = clipped_rl.first; Fy_rl = clipped_rl.second;
    Fx_rr = clipped_rr.first; Fy_rr = clipped_rr.second;

    // Sum totals: per-wheel forces already include rolling resistance; subtract aero drag here.
    const double Fy_front = Fy_fl + Fy_fr;
    const double Fy_rear  = Fy_rl + Fy_rr;
    const double Fx_sum   = Fx_fl + Fx_fr + Fx_rl + Fx_rr;

    Forces out{};
    out.Fx  = Fx_sum - Fdrag;
    out.FyF = Fy_front;
    out.FyR = Fy_rear;

    double mu_lat = 0.0;
    if (mu > 1e-6 && total_Fz > 1e-6) {
        const double Fy_total_abs = std::abs(Fy_fl) + std::abs(Fy_fr) + std::abs(Fy_rl) + std::abs(Fy_rr);
        mu_lat = Fy_total_abs / (mu * total_Fz);
    }

    last_debug_.alpha_front = alpha_f;
    last_debug_.alpha_rear  = alpha_r;
    last_debug_.fy_front    = Fy_front;
    last_debug_.fy_rear     = Fy_rear;
    last_debug_.fx_total    = out.Fx;
    last_debug_.mu_lat      = mu_lat;
    last_debug_.speed       = speed;
    last_debug_.steer       = u_in.delta;

    return out;
}

static VehicleState fDynamics(const DynamicBicycle& db, const VehicleState& x, const VehicleInput& u,
                              double Fx, double FyF, double FyR,
                              double low_weight, double brake_level){
    VehicleState xDot;
    const double m  = db.param().inertia.m;
    const double Iz = db.param().inertia.I_z;
    const double lf = db.param().kinematic.l_F;
    const double lr = db.param().kinematic.l_R;

    const double s   = std::sin(x.yaw);
    const double c   = std::cos(x.yaw);
    const double FyFtot = FyF;
    const double FyRtot = FyR;

    // World position kinematics
    xDot.position.x() = c * x.velocity.x() - s * x.velocity.y();
    xDot.position.y() = s * x.velocity.x() + c * x.velocity.y();
    xDot.yaw          = x.rotation.z();

    // Body accelerations
    xDot.velocity.x() = (x.rotation.z() * x.velocity.y() + (Fx - std::sin(u.delta) * FyFtot)) / m;
    xDot.velocity.y() = ((std::cos(u.delta) * FyFtot) + FyRtot) / m - (x.rotation.z() * x.velocity.x());

    // Yaw dynamics
    xDot.rotation.z() = (std::cos(u.delta) * FyFtot * lf - FyRtot * lr) / Iz;

    // baseline aero / carcass damping
    xDot.velocity.y() -= 0.25 * x.velocity.y();
    xDot.rotation.z() -= 0.18 * x.rotation.z();

    if (low_weight > 1e-6) {
        const double clamp_brake = std::clamp(brake_level, 0.0, 1.0);
        const double lat_damp = (6.0 + 8.0 * clamp_brake) * low_weight;
        const double yaw_damp = (3.0 + 5.0 * clamp_brake) * low_weight;
        const double vx_damp  = (1.5 + 3.0 * clamp_brake) * low_weight;
        xDot.velocity.x() -= vx_damp * x.velocity.x();
        xDot.velocity.y() -= lat_damp * x.velocity.y();
        xDot.rotation.z() -= yaw_damp * x.rotation.z();
    }

    return xDot;
}

void DynamicBicycle::updateState(VehicleState& state, const VehicleInput& inputRaw, double dt) {
    VehicleInput u = inputRaw;
    validateInput(u);

    // compute forces with v2 physics
    Forces F = computeForces(state, u, dt);

    // integrate
    const double speed = std::sqrt(state.velocity.x() * state.velocity.x() +
                                   state.velocity.y() * state.velocity.y());
    const double blend = lowSpeedBlend(speed);
    const double low_weight = 1.0 - blend;
    VehicleState xDot  = fDynamics(*this, state, u, F.Fx, F.FyF, F.FyR, low_weight, brk_eff_);
    VehicleState xNext = state + xDot * dt;

    const double speed_next = calculateMagnitude(xNext.velocity.x(), xNext.velocity.y());
    if (speed_next < 0.3) {
        double settle_brake = std::clamp(brk_eff_, 0.0, 1.0);
        double lat_tau = 0.12 + 0.25 * (1.0 - settle_brake);
        double yaw_tau = 0.15 + 0.35 * (1.0 - settle_brake);
        double vx_tau  = 0.20 + 0.30 * (1.0 - settle_brake);
        const double lat_decay = std::exp(-dt / lat_tau);
        const double yaw_decay = std::exp(-dt / yaw_tau);
        const double vx_decay  = std::exp(-dt / vx_tau);
        xNext.velocity.y() *= lat_decay;
        xNext.rotation.z() *= yaw_decay;
        xNext.velocity.x() *= vx_decay;
        if (speed_next < 0.05 && settle_brake > 0.2) {
            xNext.velocity.x() = 0.0;
            xNext.velocity.y() = 0.0;
            xNext.rotation.z() = 0.0;
        }
    }

    state = xNext;
    state.acceleration.x() = xDot.velocity.x();
    state.acceleration.y() = xDot.velocity.y();
    state.acceleration.z() = 0.0;

    validateState(state);
}
