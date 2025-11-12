#include "VehicleParam.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>
#include <vector>

// small helpers
static double getD(const YAML::Node& n, const char* k, double d = 0.0) {
  return n[k] ? n[k].as<double>() : d;
}
static int getI(const YAML::Node& n, const char* k, int d = 0) {
  return n[k] ? n[k].as<int>() : d;
}
static bool getB(const YAML::Node& n, const char* k, bool d = false) {
  return n[k] ? n[k].as<bool>() : d;
}
static std::vector<double> getVecD(const YAML::Node& n) {
  std::vector<double> v;
  if (!n || !n.IsSequence()) return v;
  v.reserve(n.size());
  for (auto&& e : n) v.push_back(e.as<double>());
  return v;
}

VehicleParam VehicleParam::loadFromFile(const std::string& yamlFile) {
  YAML::Node root = YAML::LoadFile(yamlFile);
  if (!root || !root.IsMap()) {
    throw std::runtime_error("Invalid YAML root in: " + yamlFile);
  }

  VehicleParam vp;

  // ---- inertia ----
  if (auto n = root["inertia"]) {
    vp.inertia.m   = getD(n, "m");
    vp.inertia.g   = getD(n, "g", 9.81);
    vp.inertia.I_z = getD(n, "I_z");
    vp.inertia.C_f = getD(n, "Cf");
    vp.inertia.C_r = getD(n, "Cr");
  }

  // ---- kinematics ----
  if (auto n = root["kinematics"]) {
    vp.kinematic.l          = getD(n, "l");
    vp.kinematic.b_F        = getD(n, "b_F");
    vp.kinematic.b_R        = getD(n, "b_R");
    vp.kinematic.w_front    = getD(n, "w_front", 0.5);
    vp.kinematic.axle_width = getD(n, "axle_width", 1.4);

    // Harmonize distances
    auto& K = vp.kinematic;
    if (K.l <= 0.0 && K.b_F > 0.0 && K.b_R > 0.0) {
      K.l = K.b_F + K.b_R;
    } else if (K.l > 0.0 && (K.b_F <= 0.0 || K.b_R <= 0.0)) {
      if (K.w_front > 0.0) {
        K.b_R = K.l * K.w_front;
        K.b_F = K.l - K.b_R;
      } else {
        K.b_F = 0.5 * K.l;
        K.b_R = 0.5 * K.l;
      }
    }

    if (K.w_front <= 0.0 && K.l > 0.0) {
      const double total = std::max(1e-12, K.b_F + K.b_R);
      K.w_front = K.b_R / total;
    }

    K.w_front = std::clamp(K.w_front, 0.0, 1.0);
    K.l_F = (K.b_F > 0.0)
          ? K.b_F
          : K.l * (1.0 - K.w_front);
    K.l_R = (K.b_R > 0.0)
          ? K.b_R
          : K.l * K.w_front;
  }

  // ---- tire ----
  if (auto n = root["tire"]) {
    double mu = getD(n, "tire_coefficient", 1.0);
    vp.tire.tire_coefficient = mu;

    const double B_in = getD(n, "B");
    const double C_in = getD(n, "C");
    const double D_in = getD(n, "D");
    const double E_in = getD(n, "E");

    // Normalize so D reflects unit shape, mu used as friction circle
    vp.tire.B = (mu > 1e-12) ? (B_in / mu) : B_in;
    vp.tire.C = C_in;
    vp.tire.D = D_in;      // shape gain
    vp.tire.E = E_in;
    vp.tire.radius = getD(n, "radius");
  }

  // ---- aero ----
  if (auto n = root["aero"]) {
    vp.aero.c_down = getD(n, "C_Down", 0.0);
    vp.aero.c_drag = getD(n, "C_drag", 0.0);
  }

  // ---- input_ranges ----
  if (auto ir = root["input_ranges"]) {
    if (auto a = ir["acceleration"]) {
      vp.input_ranges.acc.min = getD(a, "min");
      vp.input_ranges.acc.max = getD(a, "max");
    }
    if (auto v = ir["velocity"]) {
      vp.input_ranges.vel.min = getD(v, "min");
      vp.input_ranges.vel.max = getD(v, "max");
    }
    if (auto d = ir["steering"]) {
      vp.input_ranges.delta.min = getD(d, "min");
      vp.input_ranges.delta.max = getD(d, "max");
    }
  }

  // ---- physics ----
  if (auto n = root["physics"]) {
    vp.physics.h_cg         = getD(n, "h_cg", 0.25);
    vp.physics.c_rr         = getD(n, "c_rr", 0.012);
    vp.physics.tau_throttle = getD(n, "tau_throttle", 0.06);
    vp.physics.tau_brake    = getD(n, "tau_brake", 0.03);
  }

  // ---- brakes ----
  if (auto n = root["brakes"]) {
    vp.brakes.front_bias      = getD(n, "front_bias", 0.5);
    vp.brakes.rear_bias       = getD(n, "rear_bias", 0.5);
    vp.brakes.max_force       = getD(n, "max_force", 0.0);
    vp.brakes.regen_fraction  = getD(n, "regen_fraction", 0.0);
    vp.brakes.min_regen_speed = getD(n, "min_regen_speed", 0.0);

    double sum = vp.brakes.front_bias + vp.brakes.rear_bias;
    if (sum > 1e-12) {
      vp.brakes.front_bias /= sum;
      vp.brakes.rear_bias   = 1.0 - vp.brakes.front_bias;
    } else {
      vp.brakes.front_bias = 0.5;
      vp.brakes.rear_bias  = 0.5;
    }
  }

  // ---- powertrain ----
  if (auto n = root["powertrain"]) {
    auto& pt = vp.powertrain;

    pt.motor_count       = getI(n, "motor_count", 0);
    pt.front_motor_count = getI(n, "front_motor_count", 0);
    pt.rear_motor_count  = getI(n, "rear_motor_count", 0);
    pt.enable_regen      = getB(n, "enable_regen", true);

    pt.final_drive_ratio = getD(n, "final_drive_ratio", 1.0);
    pt.gear_efficiency   = getD(n, "gear_efficiency", 1.0);
    pt.drive_efficiency  = getD(n, "drive_efficiency", 1.0);
    pt.regen_efficiency  = getD(n, "regen_efficiency", 1.0);

    pt.max_power_kw      = getD(n, "max_power_kw", 0.0);
    pt.max_regen_kw      = getD(n, "max_regen_kw", 0.0);

    if (auto c = n["drive_curve"]) {
      pt.drive_curve.rpm    = getVecD(c["rpm"]);
      pt.drive_curve.torque = getVecD(c["torque"]);
    }
    if (auto c = n["regen_curve"]) {
      pt.regen_curve.rpm    = getVecD(c["rpm"]);
      pt.regen_curve.torque = getVecD(c["torque"]);
    }

    if (pt.drive_curve.rpm.empty()) {
      pt.drive_curve.rpm    = {0.0, 10000.0};
      pt.drive_curve.torque = {200.0, 200.0};
    }
    if (pt.regen_curve.rpm.empty()) {
      pt.regen_curve.rpm    = {0.0, 10000.0};
      pt.regen_curve.torque = {100.0, 100.0};
    }

    pt.torque_split_front   = std::max(0.0, getD(n, "torque_split_front", 0.0));
    pt.torque_split_rear    = std::max(0.0, getD(n, "torque_split_rear", 1.0));
    pt.torque_front_max_nm  = std::max(0.0, getD(n, "torque_front_max_nm", 0.0));
    pt.torque_rear_max_nm   = std::max(0.0, getD(n, "torque_rear_max_nm", 0.0));

    if (auto b = n["battery"]) {
      pt.battery.capacity_kwh          = getD(b, "capacity_kwh", 0.0);
      pt.battery.soc_init              = getD(b, "soc_init", 1.0);
      pt.battery.soc_min               = getD(b, "soc_min", 0.0);
      pt.battery.soc_max               = getD(b, "soc_max", 1.0);
      pt.battery.soc_torque_fade_start = getD(b, "soc_torque_fade_start", 0.15);
      pt.battery.soc_regen_fade_start  = getD(b, "soc_regen_fade_start", 0.95);
      pt.battery.nominal_voltage       = getD(b, "nominal_voltage", 400.0);
    }

    // Motor count reconciliation
    int total_specified = pt.front_motor_count + pt.rear_motor_count;
    if (total_specified <= 0) {
      if (pt.motor_count <= 0) pt.motor_count = 1;
      pt.rear_motor_count  = pt.motor_count;
      pt.front_motor_count = 0;
    } else {
      pt.motor_count = total_specified;
    }
    if (pt.motor_count <= 0) {
      pt.motor_count       = 1;
      pt.rear_motor_count  = std::max(pt.rear_motor_count, 1);
    }
  }

  vp.source_path = yamlFile;
  return vp;
}
