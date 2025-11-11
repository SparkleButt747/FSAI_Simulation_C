#pragma once
#include <algorithm>
#include <string>
#include <vector>

// --- Core ranges ---
struct Range { double min{0.0}; double max{0.0}; };

struct InputRanges {
  Range acc;
  Range vel;
  Range delta;
};

// --- Vehicle physical params ---
struct Inertia { double m{0.0}; double g{9.81}; double I_z{0.0}; double C_f{0.0}; double C_r{0.0}; };

struct Kinematic {
  double l{0.0};
  double b_F{0.0};
  double b_R{0.0};
  double w_front{0.5};
  double l_F{0.0};   // computed
  double l_R{0.0};   // computed
  double axle_width{1.4};
};

struct Tire {
  double tire_coefficient{1.0};
  double B{0.0};
  double C{0.0};
  double D{0.0};
  double E{0.0};
  double radius{0.0};
};

struct Aero { double c_down{0.0}; double c_drag{0.0}; };

struct Physics {
  double h_cg{0.25};
  double c_rr{0.012};
  double tau_throttle{0.06};
  double tau_brake{0.03};
};

// --- EV powertrain/brake params (data-only) ---
struct BatteryParam {
  double capacity_kwh{0.0};
  double soc_init{1.0}, soc_min{0.0}, soc_max{1.0};
  double soc_torque_fade_start{0.15};
  double soc_regen_fade_start{0.95};
  double nominal_voltage{400.0};
};

// Result of the motor/regen calculation per step
struct PowertrainStatus {
  double throttle{0.0};
  double regen{0.0};

  // Forces (+ = drive, - = brake), already at wheel(s)
  double drive_force{0.0};
  double regen_force{0.0};
  double wheel_force{0.0};

  // Axle split (for logging/validation)
  double front_drive_force{0.0};
  double rear_drive_force{0.0};
  double front_regen_force{0.0};
  double rear_regen_force{0.0};

  // Motor/battery telemetry
  double motor_torque{0.0};   // Nm (net: drive - regen)
  double motor_rpm{0.0};
  double motor_power{0.0};    // W  (net: drive - regen)
  double battery_power{0.0};  // W  (+ discharge, - charge)
  double soc{0.0};            // 0..1
};

// Brake controller request and final split
struct BrakeRequest {
  double command{0.0};        // input 0..1 (clamped)
  double regen_command{0.0};  // 0..1 portion to regen
  double target_force{0.0};   // N total braking target
};

struct BrakeStatus {
  double command{0.0};
  double regen_command{0.0};
  double regen_force{0.0};            // N taken by motor
  double front_force{0.0};            // front mechanical N
  double rear_force{0.0};             // rear mechanical N
  double total_mechanical_force{0.0}; // front + rear
};

struct TorqueCurve {
  std::vector<double> rpm;
  std::vector<double> torque;

  double sample(double x) const {
    if (rpm.empty() || torque.empty() || rpm.size() != torque.size()) return 0.0;
    if (x <= rpm.front()) return torque.front();
    if (x >= rpm.back())  return torque.back();
    auto it = std::upper_bound(rpm.begin(), rpm.end(), x);
    size_t i = static_cast<size_t>(it - rpm.begin());
    double x0 = rpm[i-1], x1 = rpm[i];
    double y0 = torque[i-1], y1 = torque[i];
    double t  = (x - x0) / std::max(1e-12, x1 - x0);
    return y0 + t*(y1 - y0);
  }
};

struct PowertrainParam {
  int    motor_count{1};
  int    front_motor_count{0};
  int    rear_motor_count{1};
  bool   enable_regen{true};
  double torque_split_front{0.0};
  double torque_split_rear{1.0};
  double torque_front_max_nm{0.0};
  double torque_rear_max_nm{195.0};
  double final_drive_ratio{1.0};
  double gear_efficiency{1.0};
  double drive_efficiency{1.0};
  double regen_efficiency{1.0};
  double max_power_kw{0.0};   // total system power cap (kW)
  double max_regen_kw{0.0};   // total system regen power cap (kW)
  TorqueCurve drive_curve;
  TorqueCurve regen_curve;
  BatteryParam battery;
};

struct BrakeParam {
  double front_bias{0.5};
  double rear_bias{0.5};
  double max_force{0.0};
  double regen_fraction{0.0};
  double min_regen_speed{0.0};
};

// --- Aggregate container ---
class VehicleParam {
public:
  Inertia inertia;
  Kinematic kinematic;
  Tire tire;
  Aero aero;
  InputRanges input_ranges;

  Physics physics;
  PowertrainParam powertrain;
  BrakeParam brakes;

  std::string source_path;    // optional: file path for diagnostics

  // Implemented in VehicleParam.cpp (yaml-cpp)
  static VehicleParam loadFromFile(const std::string& yamlFile);
};
