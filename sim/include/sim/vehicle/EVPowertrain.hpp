#pragma once
#include <sim/vehicle/VehicleParam.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

// at top of EVPowertrain.hpp (after includes)
constexpr double kPi = 3.141592653589793;

class EVMotorPowertrain {
public:
  EVMotorPowertrain() = default;
  EVMotorPowertrain(const PowertrainParam& p, double wheel_radius) { configure(p, wheel_radius); }

  void configure(const PowertrainParam& p, double wheel_radius) {
    P_ = p;
    Rw_ = std::max(1e-6, wheel_radius);
    soc_ = std::clamp(P_.battery.soc_init, P_.battery.soc_min, P_.battery.soc_max);
    int fm = std::max(0, P_.front_motor_count);
    int rm = std::max(0, P_.rear_motor_count);
    int tot = fm + rm;
    if (tot <= 0) { tot = std::max(1, P_.motor_count); rm = tot; fm = 0; }
    motor_count_ = tot;
    front_share_ = (tot>0)? double(fm)/double(tot) : 0.0;
    rear_share_  = 1.0 - front_share_;
  }

  double soc() const { return soc_; }

  PowertrainStatus compute(double throttle01, double regen01, double vx, double dt) {
    throttle01 = std::clamp(throttle01, 0.0, 1.0);
    regen01    = std::clamp(regen01,    0.0, 1.0);
    if (!P_.enable_regen) regen01 = 0.0;
    if (throttle01>0.0 && regen01>0.0) regen01 = 0.0;

    const double wheel_omega = double(vx) / Rw_;
    const double motor_omega = wheel_omega * P_.final_drive_ratio;
    const double motor_rpm   = motor_omega * 60.0 / (2.0 * kPi);

    double drive_Tm=0.0, drive_Pm=0.0, drive_Pb=0.0;
    double regen_Tm=0.0, regen_Pm=0.0, regen_Pb=0.0;

    if (throttle01>0.0) driveSide(throttle01, motor_rpm, motor_omega, dt, drive_Tm, drive_Pm, drive_Pb);
    if (regen01>0.0)    regenSide  (regen01, motor_rpm, motor_omega, dt, regen_Tm, regen_Pm, regen_Pb);

    // Wheel torques (final drive & gear eff)
    const double T_w_drive = drive_Tm * P_.final_drive_ratio * P_.gear_efficiency;
    const double T_w_regen = regen_Tm * P_.final_drive_ratio * P_.gear_efficiency;

    const double drive_F  = T_w_drive / Rw_;
    const double regen_F  = T_w_regen / Rw_;
    const double front_drive = drive_F * front_share_;
    const double rear_drive  = drive_F * rear_share_;
    const double front_regen = regen_F * front_share_;
    const double rear_regen  = regen_F * rear_share_;

    const double motor_T   = drive_Tm - regen_Tm;
    const double motor_P   = drive_Pm - regen_Pm;
    double battery_P       = drive_Pb + regen_Pb;

    integrateSOC(battery_P, dt);

    PowertrainStatus S;
    S.throttle = throttle01; S.regen = regen01;
    S.drive_force = drive_F; S.regen_force = regen_F;
    S.wheel_force = drive_F - regen_F;
    S.front_drive_force = front_drive; S.rear_drive_force = rear_drive;
    S.front_regen_force = front_regen; S.rear_regen_force = rear_regen;
    S.motor_torque = motor_T; S.motor_rpm = motor_rpm;
    S.motor_power  = motor_P; S.battery_power = battery_P;
    S.soc = soc_;
    return S;
  }

private:
  PowertrainParam P_{};
  double Rw_{0.3};
  int   motor_count_{1};
  double front_share_{0.0}, rear_share_{1.0};
  double soc_{1.0};

  static inline double safeEff(double e){ return (e>1e-6)? e : 1.0; }

  double torqueFromPowerLimit(double motor_omega, double max_power_kw, int motors) const {
    double P = std::max(0.0, max_power_kw) * 1000.0 * std::max(1, motors);
    if (P<=0.0 || motor_omega<=1e-6) return std::numeric_limits<double>::infinity();
    return P / motor_omega;
  }

  double driveSocScale() const {
    const auto& b = P_.battery;
    if (b.soc_torque_fade_start <= b.soc_min) return (soc_>b.soc_min)?1.0:0.0;
    if (soc_ <= b.soc_min) return 0.0;
    if (soc_ >= b.soc_torque_fade_start) return 1.0;
    double span = b.soc_torque_fade_start - b.soc_min;
    return std::clamp((soc_ - b.soc_min)/std::max(1e-6, span), 0.0, 1.0);
  }

  double regenSocScale() const {
    const auto& b = P_.battery;
    if (b.soc_regen_fade_start >= b.soc_max) return (soc_<b.soc_max)?1.0:0.0;
    if (soc_ >= b.soc_max) return 0.0;
    if (soc_ <= b.soc_regen_fade_start) return 1.0;
    double span = b.soc_max - b.soc_regen_fade_start;
    return std::clamp((b.soc_max - soc_) / std::max(1e-6, span), 0.0, 1.0);
  }

  void driveSide(double cmd, double motor_rpm, double motor_omega, double dt,
                 double& out_Tm, double& out_Pm, double& out_Pb) {
    const double per_motor = std::max(0.0, P_.drive_curve.sample(motor_rpm));
    double T_limit = per_motor * std::max(1, motor_count_);
    double T_req   = std::clamp(cmd, 0.0, 1.0) * T_limit;

    T_limit *= driveSocScale();
    if (T_req > T_limit) T_req = T_limit;

    double T_power = torqueFromPowerLimit(motor_omega, P_.max_power_kw, motor_count_);
    if (T_req > T_power) T_req = T_power;

    T_req = std::max(0.0, T_req);
    double Pm = T_req * std::max(0.0, motor_omega);
    double Pb = Pm / safeEff(P_.drive_efficiency);

    scaleBatteryToBounds(Pb, dt);
    double scale = (Pm>1e-9) ? std::clamp(Pb * safeEff(P_.drive_efficiency) / Pm, 0.0, 1.0) : 1.0;
    out_Tm = T_req * scale; out_Pm = Pm * scale; out_Pb = Pb;
  }

  void regenSide(double cmd, double motor_rpm, double motor_omega, double dt,
                 double& out_Tm, double& out_Pm, double& out_Pb) {
    const double per_motor = std::max(0.0, P_.regen_curve.sample(motor_rpm));
    double T_limit = per_motor * std::max(1, motor_count_);
    double T_req   = std::clamp(cmd, 0.0, 1.0) * T_limit;

    T_limit *= regenSocScale();
    if (T_req > T_limit) T_req = T_limit;

    double T_power = torqueFromPowerLimit(motor_omega, P_.max_regen_kw, motor_count_);
    if (T_req > T_power) T_req = T_power;

    T_req = std::max(0.0, T_req);
    double Pm = T_req * std::max(0.0, motor_omega);
    double Pb = -Pm * P_.regen_efficiency;

    scaleBatteryToBounds(Pb, dt);
    double scale = (Pm>1e-9) ? std::clamp((-Pb) / (Pm * P_.regen_efficiency), 0.0, 1.0) : 1.0;
    out_Tm = T_req * scale; out_Pm = Pm * scale; out_Pb = Pb;
  }

  void integrateSOC(double battery_power_W, double dt) {
    double capacity_J = std::max(0.0, P_.battery.capacity_kwh) * 3.6e6;
    if (capacity_J <= 0.0 || dt <= 0.0) return;
    double delta = -battery_power_W * dt / capacity_J;
    soc_ = std::clamp(soc_ + delta, P_.battery.soc_min, P_.battery.soc_max);
  }

  void scaleBatteryToBounds(double& battery_power_W, double dt) const {
    double capacity_J = std::max(0.0, P_.battery.capacity_kwh) * 3.6e6;
    if (capacity_J <= 0.0 || dt <= 0.0) { battery_power_W = 0.0; return; }
    if (battery_power_W == 0.0) return;

    const auto& b = P_.battery;
    if (battery_power_W > 0.0) {
      double avail = std::max(0.0, (soc_ - b.soc_min)) * capacity_J;
      if (avail <= 0.0) { battery_power_W = 0.0; return; }
      double maxP = avail / dt;
      if (battery_power_W > maxP) battery_power_W = maxP;
    } else {
      double room = std::max(0.0, (b.soc_max - soc_)) * capacity_J;
      if (room <= 0.0) { battery_power_W = 0.0; return; }
      double maxP = room / dt;
      if (std::fabs(battery_power_W) > maxP) battery_power_W = -maxP;
    }
  }
};
