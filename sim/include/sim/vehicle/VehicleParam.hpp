#pragma once

#include <algorithm>
#include <string>
#include <vector>

// -------- Core ranges --------

struct Range {
  double min{0.0};
  double max{0.0};
};

struct InputRanges {
  Range acc;
  Range vel;
  Range delta;
};

// -------- Vehicle physical params --------

struct Inertia {
  double m{0.0};      // mass [kg]
  double g{9.81};     // gravity [m/s^2]
  double I_z{0.0};    // yaw inertia [kg m^2]

  // Optional linear cornering stiffness (for alternative models / logging)
  double C_f{0.0};    // front axle [N/rad]
  double C_r{0.0};    // rear axle [N/rad]
};

struct Kinematic {
  double l{0.0};          // wheelbase [m]
  double b_F{0.0};        // CG -> front axle [m]
  double b_R{0.0};        // CG -> rear axle [m]
  double w_front{0.5};    // static front weight fraction [-]
  double l_F{0.0};        // derived: CG->front [m]
  double l_R{0.0};        // derived: CG->rear [m]
  double axle_width{1.4}; // track width [m]
};

struct Tire {
  /**
   * Nonlinear lateral model parameters (Pacejka-like).
   *
   * Loader applies:
   *   B_eff = B / tire_coefficient
   *   D_eff = D * tire_coefficient
   * And uses tire_coefficient as μ for friction circle.
   */
  double tire_coefficient{1.0}; // effective μ for friction circle
  double B{0.0};
  double C{0.0};
  double D{0.0};
  double E{0.0};
  double radius{0.0};           // [m]
};

struct Aero {
  double c_down{0.0}; // F_down = c_down * v^2
  double c_drag{0.0}; // F_drag = c_drag * v^2
};

struct Physics {
  double h_cg{0.25};      // [m] CG height
  double c_rr{0.012};     // rolling resistance coefficient
  double tau_throttle{0.06}; // [s] 1st-order lag
  double tau_brake{0.03};    // [s] 1st-order lag
};

// -------- EV powertrain / brakes (data-only) --------

struct BatteryParam {
  double capacity_kwh{0.0};
  double soc_init{1.0};
  double soc_min{0.0};
  double soc_max{1.0};
  double soc_torque_fade_start{0.15};
  double soc_regen_fade_start{0.95};
  double nominal_voltage{400.0};
};

struct TorqueCurve {
  std::vector<double> rpm;
  std::vector<double> torque;

  double sample(double x) const {
    if (rpm.empty() || torque.empty() || rpm.size() != torque.size()) return 0.0;
    if (x <= rpm.front()) return torque.front();
    if (x >= rpm.back())  return torque.back();

    auto it = std::upper_bound(rpm.begin(), rpm.end(), x);
    const size_t i = static_cast<size_t>(it - rpm.begin());
    const double x0 = rpm[i - 1], x1 = rpm[i];
    const double y0 = torque[i - 1], y1 = torque[i];
    const double t  = (x - x0) / std::max(1e-12, x1 - x0);
    return y0 + t * (y1 - y0);
  }
};

struct PowertrainParam {
  int    motor_count{1};
  int    front_motor_count{0};
  int    rear_motor_count{1};
  bool   enable_regen{true};

  // Optional torque split hints (normalized in code)
  double torque_split_front{0.0};
  double torque_split_rear{1.0};

  double torque_front_max_nm{0.0};
  double torque_rear_max_nm{0.0};

  double final_drive_ratio{1.0};
  double gear_efficiency{1.0};
  double drive_efficiency{1.0};
  double regen_efficiency{1.0};

  double max_power_kw{0.0};   // total drive power cap
  double max_regen_kw{0.0};   // total regen power cap

  TorqueCurve drive_curve;    // per-motor torque map
  TorqueCurve regen_curve;    // per-motor regen map

  BatteryParam battery;
};

struct BrakeParam {
  double front_bias{0.5};     // nominal fraction
  double rear_bias{0.5};
  double max_force{0.0};      // [N] max total mech brake force
  double regen_fraction{0.0}; // fraction of cmd to use for regen
  double min_regen_speed{0.0}; // [m/s] regen enabled above
};

// -------- Status structs (runtime outputs) --------

struct PowertrainStatus {
  double throttle{0.0};
  double regen{0.0};

  // Forces at wheels (+ drive, - brake/regen)
  double drive_force{0.0};
  double regen_force{0.0};
  double wheel_force{0.0};

  // Axle split
  double front_drive_force{0.0};
  double rear_drive_force{0.0};
  double front_regen_force{0.0};
  double rear_regen_force{0.0};

  // Motor/battery telemetry
  double motor_torque{0.0};    // net [Nm]
  double motor_rpm{0.0};
  double motor_power{0.0};     // net [W]
  double battery_power{0.0};   // +discharge, -charge [W]
  double soc{0.0};             // 0..1
};

struct BrakeRequest {
  double command{0.0};        // 0..1 (clamped)
  double regen_command{0.0};  // 0..1 portion to regen
  double target_force{0.0};   // [N] total brake target
};

struct BrakeStatus {
  double command{0.0};
  double regen_command{0.0};

  double regen_force{0.0};            // [N] via motor(s)
  double front_force{0.0};            // [N] mech front
  double rear_force{0.0};             // [N] mech rear
  double total_mechanical_force{0.0}; // front + rear
};

// -------- Aggregate container --------

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

