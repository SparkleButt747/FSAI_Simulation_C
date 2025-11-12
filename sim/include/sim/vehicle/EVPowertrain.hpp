#pragma once
#include "VehicleParam.hpp"

class EVMotorPowertrain {
public:
  EVMotorPowertrain() = default;
  EVMotorPowertrain(const PowertrainParam& p, double wheel_radius) { configure(p, wheel_radius); }

  void configure(const PowertrainParam& p, double wheel_radius);

  double soc() const { return soc_; }

  PowertrainStatus compute(double throttle01,
                           double regen01,
                           double vx,
                           double dt);

private:
  PowertrainParam P_{};
  double Rw_{0.3};
  int    motor_count_{1};
  double front_share_{0.0};
  double rear_share_{1.0};
  double soc_{1.0};

  static inline double safeEff(double e) { return (e > 1e-6) ? e : 1.0; }
  double torqueFromPowerLimit(double motor_omega, double max_power_kw) const;
  double driveSocScale() const;
  double regenSocScale() const;

  void driveSide(double cmd, double motor_rpm, double motor_omega, double dt,
                 double& out_Tm, double& out_Pm, double& out_Pb);
  void regenSide(double cmd, double motor_rpm, double motor_omega, double dt,
                 double& out_Tm, double& out_Pm, double& out_Pb);

  void integrateSOC(double battery_power_W, double dt);
  void scaleBatteryToBounds(double& battery_power_W, double dt) const;
};
