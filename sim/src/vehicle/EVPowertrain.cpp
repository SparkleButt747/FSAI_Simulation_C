#include "EVPowertrain.hpp"
#include "SanityChecks.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using fsai::sim::sanity::fail;

static constexpr double kPi = 3.141592653589793;

void EVMotorPowertrain::configure(const PowertrainParam& p, double wheel_radius) {
  P_  = p;
  Rw_ = std::max(1e-6, wheel_radius);

  soc_ = std::clamp(P_.battery.soc_init, P_.battery.soc_min, P_.battery.soc_max);

  int fm  = std::max(0, P_.front_motor_count);
  int rm  = std::max(0, P_.rear_motor_count);
  int tot = fm + rm;

  if (tot <= 0) {
    tot = std::max(1, P_.motor_count);
    fm = 0; rm = tot;
  }
  motor_count_ = tot;

  double fs = (tot > 0) ? static_cast<double>(fm) / static_cast<double>(tot) : 0.0;
  double rs = 1.0 - fs;

  if (P_.torque_split_front > 0.0 || P_.torque_split_rear > 0.0) {
    double f_raw = std::max(0.0, P_.torque_split_front);
    double r_raw = std::max(0.0, P_.torque_split_rear);
    double sum = f_raw + r_raw; if (sum <= 1e-9) sum = 1.0;
    f_raw /= sum; r_raw /= sum;
    if (fm == 0) { f_raw = 0.0; r_raw = 1.0; }
    if (rm == 0) { f_raw = 1.0; r_raw = 0.0; }
    double norm = f_raw + r_raw; if (norm > 1e-9) { fs = f_raw / norm; rs = r_raw / norm; }
  }

  front_share_ = std::clamp(fs, 0.0, 1.0);
  rear_share_  = 1.0 - front_share_;

  if (!(motor_count_ >= 1)) fail("EVPowertrain::configure", "motor_count < 1", motor_count_);
}

double EVMotorPowertrain::torqueFromPowerLimit(double motor_omega, double max_power_kw) const {
  const double P = std::max(0.0, max_power_kw) * 1000.0;
  if (P <= 0.0 || motor_omega <= 1e-6) return std::numeric_limits<double>::infinity();
  return P / motor_omega;
}

double EVMotorPowertrain::driveSocScale() const {
  const auto& b = P_.battery;
  if (b.soc_torque_fade_start <= b.soc_min) return (soc_ > b.soc_min) ? 1.0 : 0.0;
  if (soc_ <= b.soc_min) return 0.0;
  if (soc_ >= b.soc_torque_fade_start) return 1.0;
  const double span = b.soc_torque_fade_start - b.soc_min;
  return std::clamp((soc_ - b.soc_min) / std::max(1e-6, span), 0.0, 1.0);
}

double EVMotorPowertrain::regenSocScale() const {
  const auto& b = P_.battery;
  if (b.soc_regen_fade_start >= b.soc_max) return (soc_ < b.soc_max) ? 1.0 : 0.0;
  if (soc_ >= b.soc_max) return 0.0;
  if (soc_ <= b.soc_regen_fade_start) return 1.0;
  const double span = b.soc_max - b.soc_regen_fade_start;
  return std::clamp((b.soc_max - soc_) / std::max(1e-6, span), 0.0, 1.0);
}

void EVMotorPowertrain::driveSide(double cmd, double motor_rpm, double motor_omega, double dt,
                                  double& out_Tm, double& out_Pm, double& out_Pb) {
  cmd = std::clamp(cmd, 0.0, 1.0);
  const double per_motor = std::max(0.0, P_.drive_curve.sample(motor_rpm));
  double T_limit = per_motor * std::max(1, motor_count_);
  T_limit *= driveSocScale();

  double T_req = cmd * T_limit;
  const double T_power = torqueFromPowerLimit(motor_omega, P_.max_power_kw);
  if (T_req > T_power) T_req = T_power;
  T_req = std::max(0.0, T_req);

  const double Pm = T_req * std::max(0.0, motor_omega);
  double Pb = Pm / safeEff(P_.drive_efficiency);

  scaleBatteryToBounds(Pb, dt);

  double scale = 1.0;
  if (Pm > 1e-9) {
    const double Pm_allowed = Pb * safeEff(P_.drive_efficiency);
    scale = std::clamp(Pm_allowed / Pm, 0.0, 1.0);
  }

  out_Tm = T_req * scale;
  out_Pm = Pm * scale;
  out_Pb = Pb;

  if (!(out_Tm >= 0.0)) fail("EVPowertrain::driveSide", "out_Tm < 0", out_Tm);
}

void EVMotorPowertrain::regenSide(double cmd, double motor_rpm, double motor_omega, double dt,
                                  double& out_Tm, double& out_Pm, double& out_Pb) {
  cmd = std::clamp(cmd, 0.0, 1.0);
  const double per_motor = std::max(0.0, P_.regen_curve.sample(motor_rpm));
  double T_limit = per_motor * std::max(1, motor_count_);
  T_limit *= regenSocScale();

  double T_req = cmd * T_limit;
  const double T_power = torqueFromPowerLimit(motor_omega, P_.max_regen_kw);
  if (T_req > T_power) T_req = T_power;
  T_req = std::max(0.0, T_req);

  const double Pm = T_req * std::max(0.0, motor_omega);
  double Pb = -Pm * P_.regen_efficiency; // charging -> negative battery power

  scaleBatteryToBounds(Pb, dt);

  double scale = 1.0;
  if (Pm > 1e-9) {
    const double Pm_allowed = -Pb / safeEff(P_.regen_efficiency);
    scale = std::clamp(Pm_allowed / Pm, 0.0, 1.0);
  }

  out_Tm = T_req * scale;
  out_Pm = Pm * scale;
  out_Pb = Pb;

  if (!(out_Tm >= 0.0)) fail("EVPowertrain::regenSide", "out_Tm < 0", out_Tm);
}

void EVMotorPowertrain::integrateSOC(double battery_power_W, double dt) {
  const double capacity_J = std::max(0.0, P_.battery.capacity_kwh) * 3.6e6;
  if (capacity_J <= 0.0 || dt <= 0.0) return;
  const double delta = -battery_power_W * dt / capacity_J; // +P -> discharge -> SOC down
  soc_ = std::clamp(soc_ + delta, P_.battery.soc_min, P_.battery.soc_max);
}

void EVMotorPowertrain::scaleBatteryToBounds(double& battery_power_W, double dt) const {
  const double capacity_J = std::max(0.0, P_.battery.capacity_kwh) * 3.6e6;
  if (capacity_J <= 0.0 || dt <= 0.0) { battery_power_W = 0.0; return; }
  if (battery_power_W == 0.0) return;

  const auto& b = P_.battery;
  if (battery_power_W > 0.0) {
    const double avail = std::max(0.0, (soc_ - b.soc_min)) * capacity_J;
    if (avail <= 0.0) { battery_power_W = 0.0; return; }
    const double maxP = avail / dt;
    if (battery_power_W > maxP) battery_power_W = maxP;
  } else {
    const double room = std::max(0.0, (b.soc_max - soc_)) * capacity_J;
    if (room <= 0.0) { battery_power_W = 0.0; return; }
    const double maxP = room / dt;
    if (std::fabs(battery_power_W) > maxP) battery_power_W = -maxP;
  }
}

PowertrainStatus EVMotorPowertrain::compute(double throttle01, double regen01, double vx, double dt) {
  if (!std::isfinite(throttle01)) fail("EVPowertrain::compute", "throttle !finite", throttle01);
  if (!std::isfinite(regen01))    fail("EVPowertrain::compute", "regen !finite", regen01);
  if (!std::isfinite(vx))         fail("EVPowertrain::compute", "vx !finite", vx);
  if (!(dt>0.0 && std::isfinite(dt) && dt<1.0)) fail("EVPowertrain::compute","dt invalid",dt);

  throttle01 = std::clamp(throttle01, 0.0, 1.0);
  regen01    = std::clamp(regen01,    0.0, 1.0);
  if (!P_.enable_regen) regen01 = 0.0;
  if (throttle01 > 0.0 && regen01 > 0.0) regen01 = 0.0;

  const double wheel_omega = vx / Rw_;
  const double motor_omega = wheel_omega * P_.final_drive_ratio;
  const double motor_rpm   = motor_omega * 60.0 / (2.0 * kPi);

  double drive_Tm=0, drive_Pm=0, drive_Pb=0;
  double regen_Tm=0, regen_Pm=0, regen_Pb=0;

  if (throttle01 > 0.0) driveSide(throttle01, motor_rpm, motor_omega, dt, drive_Tm, drive_Pm, drive_Pb);
  if (regen01    > 0.0) regenSide(regen01,    motor_rpm, motor_omega, dt, regen_Tm, regen_Pm, regen_Pb);

  const double motor_T = drive_Tm - regen_Tm;
  const double motor_P = drive_Pm - regen_Pm;
  double battery_P = drive_Pb + regen_Pb;

  integrateSOC(battery_P, dt);

  const double T_w_drive = drive_Tm * P_.final_drive_ratio * P_.gear_efficiency;
  const double T_w_regen = regen_Tm * P_.final_drive_ratio * P_.gear_efficiency;

  const double drive_F = T_w_drive / Rw_;
  const double regen_F = T_w_regen / Rw_;

  const double front_drive = drive_F * front_share_;
  const double rear_drive  = drive_F * rear_share_;
  const double front_regen = regen_F * front_share_;
  const double rear_regen  = regen_F * rear_share_;

  PowertrainStatus S{};
  S.throttle = throttle01; S.regen = regen01;
  S.drive_force = drive_F; S.regen_force = regen_F; S.wheel_force = drive_F - regen_F;
  S.front_drive_force = front_drive; S.rear_drive_force = rear_drive;
  S.front_regen_force = front_regen; S.rear_regen_force = rear_regen;
  S.motor_torque = motor_T; S.motor_rpm = motor_rpm; S.motor_power = motor_P;
  S.battery_power = battery_P; S.soc = soc_;

  // Sanity: non-negatives and power cap headroom
  if (S.front_drive_force < -1e-9 || S.rear_drive_force < -1e-9) fail("EVPowertrain::compute","neg drive force", S.front_drive_force+S.rear_drive_force);
  if (S.front_regen_force < -1e-9 || S.rear_regen_force < -1e-9) fail("EVPowertrain::compute","neg regen force", S.front_regen_force+S.rear_regen_force);
  if (P_.max_power_kw > 0.0 && motor_P > 1.25 * P_.max_power_kw * 1000.0) fail("EVPowertrain::compute","motor power >125% cap", motor_P);

  return S;
}
