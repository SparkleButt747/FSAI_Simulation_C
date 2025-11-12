#pragma once
#include <cmath>
#include <cfloat>
#include <stdexcept>
#include <string>
#include <sstream>
#include <algorithm>
#include <limits>
#include <vector>
#include <type_traits>

#include <Eigen/Dense>

#include "VehicleParam.hpp"
#include "VehicleState.hpp"
#include "VehicleInput.hpp"
#include "WheelsInfo.h"

namespace fsai { namespace sim { namespace sanity {

struct Limits {
  // generic numeric tolerance
  double eps               {1e-9};

  // velocities [m/s]
  double v_abs_max         {180.0};     // absolute component ceiling
  double v_speed_max       {180.0};     // sqrt(vx^2+vy^2)

  // accelerations [m/s^2]
  double ax_abs_max        {50.0};
  double ay_abs_max        {50.0};

  // yaw / angular rates [rad], [rad/s], [rad/s^2]
  bool   wrap_yaw_to_pi    {true};
  double yaw_abs_max       {1e6};       // no hard clamp if wrap=true
  double r_abs_max         {30.0};
  double rdot_abs_max      {500.0};

  // slip angles [rad]
  double slip_abs_max      {1.57079632679 - 1e-3};

  // forces [N]
  double fx_abs_max        {2.0e6};
  double fy_abs_max        {2.0e6};
  double fsum_abs_max      {3.0e6};

  // powertrain
  double soc_min           {0.0};
  double soc_max           {1.0};

  // steering [rad]
  double steering_abs_max  {2.0};

  // wheels rpm (if used elsewhere)
  double wheel_rpm_abs_max {2.0e4};
};


// ------- failure utility -------
template <class T>
[[noreturn]] inline void fail(const char* where, const char* what, const T& v) {
  std::ostringstream oss;
  oss << "[SANITY] " << where << ": " << what << " (val=" << v << ")";
  std::fprintf(stderr, "%s\n", oss.str().c_str());
  throw std::runtime_error(oss.str());
}

[[noreturn]] inline void fail(const char* where, const char* what) {
  std::ostringstream oss;
  oss << "[SANITY] " << where << ": " << what;
  std::fprintf(stderr, "%s\n", oss.str().c_str());
  throw std::runtime_error(oss.str());
}

// ---------- basic finite checks ----------

inline bool finite(double x) { return std::isfinite(x); }

inline void checkFinite(const char* where, const char* field, double v)
{
  if (!finite(v)) fail(where, field, v);
}

inline void checkFiniteVec3(const char* where, const char* field, const Eigen::Vector3d& v)
{
  if (!finite(v.x()) || !finite(v.y()) || !finite(v.z())) {
    fail(where, field);
  }
}

// ---------- param sanity ----------
inline void checkParam(const VehicleParam& p, const Limits& L, const char* where)
{
  if (p.inertia.m <= 1.0)                fail(where, "mass too small", p.inertia.m);
  if (p.inertia.g < 5.0 || p.inertia.g > 20.0) fail(where, "gravity out of range", p.inertia.g);
  if (p.inertia.I_z <= 0.0)              fail(where, "I_z <= 0", p.inertia.I_z);

  if (p.kinematic.l <= 0.3)              fail(where, "wheelbase too small", p.kinematic.l);
  if (p.kinematic.axle_width <= 0.3)     fail(where, "axle_width too small", p.kinematic.axle_width);
  if (p.kinematic.w_front < 0.0 || p.kinematic.w_front > 1.0)
    fail(where, "w_front not in [0,1]", p.kinematic.w_front);

  if (p.tire.radius <= 0.05 || p.tire.radius > 1.0)
    fail(where, "tire radius out of range", p.tire.radius);

  if (p.tire.tire_coefficient <= 0.1 || p.tire.tire_coefficient > 3.5)
    fail(where, "tire mu out of range", p.tire.tire_coefficient);

  if (p.brakes.max_force < 0.0)          fail(where, "brake max_force < 0", p.brakes.max_force);
  if (p.brakes.front_bias < 0.0 || p.brakes.front_bias > 1.0)
    fail(where, "front_bias not in [0,1]", p.brakes.front_bias);

  if (p.powertrain.final_drive_ratio <= 0.0)
    fail(where, "final_drive_ratio <= 0", p.powertrain.final_drive_ratio);
  if (p.powertrain.motor_count <= 0)     fail(where, "motor_count <= 0", p.powertrain.motor_count);

  // torque curves monotonic rpm
  auto mono = [](const std::vector<double>& v)->bool{
    for (size_t i=1;i<v.size();++i) if (v[i] < v[i-1]) return false; return true;
  };
  if (!p.powertrain.drive_curve.rpm.empty() && !mono(p.powertrain.drive_curve.rpm))
    fail(where, "drive_curve.rpm not nondecreasing");
  if (!p.powertrain.regen_curve.rpm.empty() && !mono(p.powertrain.regen_curve.rpm))
    fail(where, "regen_curve.rpm not nondecreasing");
}

// ------- input sanity -------


inline void checkInput(const VehicleParam& p,
                       const VehicleInput& u,
                       const char* where,
                       const Limits& L = Limits{})
{
  checkFinite(where, "acc",   u.acc);
  checkFinite(where, "vel",   u.vel);
  checkFinite(where, "delta", u.delta);

  const double amin = p.input_ranges.acc.min;
  const double amax = p.input_ranges.acc.max;
  const double vmin = p.input_ranges.vel.min;
  const double vmax = p.input_ranges.vel.max;

  // steering range may be absent in older structs; fallback to Limits
  double dmin = L.steering_abs_max * -1.0;
  double dmax = L.steering_abs_max;
  // Check if the project uses InputRanges::delta and not ::steering
  dmin = p.input_ranges.delta.min;
  dmax = p.input_ranges.delta.max;

  if (u.acc   < amin - L.eps || u.acc   > amax + L.eps) fail(where, "acc out of range",   u.acc);
  if (u.vel   < vmin - L.eps || u.vel   > vmax + L.eps) fail(where, "vel out of range",   u.vel);
  if (u.delta < dmin - L.eps || u.delta > dmax + L.eps) fail(where, "delta out of range", u.delta);
}


// ------- state sanity -------

inline void wrapYaw(VehicleState& s)
{
  double y = s.yaw;
  if (!std::isfinite(y)) return;
  while (y >  M_PI) y -= 2.0*M_PI;
  while (y < -M_PI) y += 2.0*M_PI;
  s.yaw = y;
}

inline void checkState(const VehicleState& s,
                       const char* where,
                       const Limits& L = Limits{})
{
  checkFiniteVec3(where, "position",     s.position);
  checkFiniteVec3(where, "velocity",     s.velocity);
  checkFiniteVec3(where, "rotation",     s.rotation);
  checkFiniteVec3(where, "acceleration", s.acceleration);
  checkFinite(where, "yaw", s.yaw);

  const double speed = std::sqrt(s.velocity.x()*s.velocity.x()
                               + s.velocity.y()*s.velocity.y());
  if (std::abs(s.velocity.x()) > L.v_abs_max) fail(where, "vx too large", s.velocity.x());
  if (std::abs(s.velocity.y()) > L.v_abs_max) fail(where, "vy too large", s.velocity.y());
  if (speed > L.v_speed_max)                  fail(where, "speed too large", speed);

  if (std::abs(s.rotation.z()) > L.r_abs_max) fail(where, "yaw rate too large", s.rotation.z());
}


inline void checkState(const VehicleParam& p,
                       const VehicleState& x,
                       const char* where)
{
  (void)p;
  if (!finite(x.position.x()) || !finite(x.position.y()) || !finite(x.position.z())) fail(where, "position !finite");
  if (!finite(x.velocity.x()) || !finite(x.velocity.y()) || !finite(x.velocity.z())) fail(where, "velocity !finite");
  if (!finite(x.rotation.x()) || !finite(x.rotation.y()) || !finite(x.rotation.z())) fail(where, "rotation !finite");
  if (!finite(x.acceleration.x()) || !finite(x.acceleration.y()) || !finite(x.acceleration.z())) fail(where, "accel !finite");
  if (!finite(x.yaw)) fail(where, "yaw !finite");

  // Optional: guard insane magnitudes (prevents blow-ups from propagating)
  const double vmax = 200.0; // m/s cap for sanity
  if (std::fabs(x.velocity.x()) > 5*vmax || std::fabs(x.velocity.y()) > 5*vmax)
    fail(where, "velocity magnitude absurd");
}

// ---------- slip sanity ----------

inline void checkSlip(double alpha,
                      const char* where,
                      const Limits& L = Limits{})
{
  checkFinite(where, "alpha", alpha);
  if (std::abs(alpha) > L.slip_abs_max) fail(where, "slip angle |alpha| too large", alpha);
}

inline void checkSlipPair(double aF, double aR, const char* where, const Limits& L = Limits{})
{
  checkSlip(aF, where, L);
  checkSlip(aR, where, L);
}


// ------- force checks -------

struct ForceSummary {
  double Fx{0.0}, FyF{0.0}, FyR{0.0};
};

inline void checkForces(const ForceSummary& f, const char* where, const Limits& L = Limits{})
{
  checkFinite(where, "Fx",  f.Fx);
  checkFinite(where, "FyF", f.FyF);
  checkFinite(where, "FyR", f.FyR);

  if (std::abs(f.Fx)  > L.fx_abs_max) fail(where, "Fx too large",  f.Fx);
  if (std::abs(f.FyF) > L.fy_abs_max) fail(where, "FyF too large", f.FyF);
  if (std::abs(f.FyR) > L.fy_abs_max) fail(where, "FyR too large", f.FyR);

  const double sumLat = std::abs(f.FyF) + std::abs(f.FyR);
  const double sumTot = std::abs(f.Fx) + sumLat;
  if (sumTot > L.fsum_abs_max) fail(where, "sum |F| too large", sumTot);
}

inline void checkForces(const VehicleParam& p,
                        double Fx, double FyF, double FyR,
                        const char* where)
{
  (void)p;
  if (!finite(Fx) || !finite(FyF) || !finite(FyR)) fail(where, "forces !finite");
  // Extremely loose physical envelope to catch NaN/overflow only
  const double Fmax = 5e6;
  if (std::fabs(Fx)  > Fmax) fail(where, "Fx overflow", Fx);
  if (std::fabs(FyF) > Fmax) fail(where, "FyF overflow", FyF);
  if (std::fabs(FyR) > Fmax) fail(where, "FyR overflow", FyR);
}

// ---------- derivatives sanity (xDot) ----------

inline void checkDerivatives(const VehicleState& xDot,
                             const char* where,
                             const Limits& L = Limits{})
{
  checkFiniteVec3(where, "xDot.position",     xDot.position);
  checkFiniteVec3(where, "xDot.velocity",     xDot.velocity);
  checkFiniteVec3(where, "xDot.rotation",     xDot.rotation);
  checkFiniteVec3(where, "xDot.acceleration", xDot.acceleration);

  if (std::abs(xDot.velocity.x()) > L.ax_abs_max) fail(where, "ax too large", xDot.velocity.x());
  if (std::abs(xDot.velocity.y()) > L.ay_abs_max) fail(where, "ay too large", xDot.velocity.y());
  if (std::abs(xDot.rotation.z()) > L.rdot_abs_max) fail(where, "yaw accel too large", xDot.rotation.z());
}

// ---------- powertrain & brake sanity ----------

inline void checkPowertrain(const PowertrainStatus& ps,
                            const char* where,
                            const Limits& L = Limits{})
{
  checkFinite(where, "throttle", ps.throttle);
  checkFinite(where, "regen",    ps.regen);
  checkFinite(where, "drive_force", ps.drive_force);
  checkFinite(where, "regen_force", ps.regen_force);
  checkFinite(where, "wheel_force", ps.wheel_force);
  checkFinite(where, "motor_torque", ps.motor_torque);
  checkFinite(where, "motor_rpm",    ps.motor_rpm);
  checkFinite(where, "motor_power",  ps.motor_power);
  checkFinite(where, "battery_power",ps.battery_power);
  checkFinite(where, "soc",          ps.soc);

  if (ps.throttle < -L.eps || ps.throttle > 1.0 + L.eps) fail(where, "throttle outside [0,1]", ps.throttle);
  if (ps.regen    < -L.eps || ps.regen    > 1.0 + L.eps) fail(where, "regen outside [0,1]",    ps.regen);
  if (ps.soc      < L.soc_min - 1e-4 || ps.soc > L.soc_max + 1e-4) fail(where, "SOC out of [0,1]", ps.soc);

  // No simultaneous positive drive and regen wheel contributions
  if (ps.drive_force > 1e-6 && ps.regen_force > 1e-6) fail(where, "drive and regen both positive");

  // Split consistency
  auto nonneg = [&](double v, const char* n){ if (v < -1e-6 || !finite(v)) fail(where, n, v); };
  nonneg(ps.front_drive_force, "front_drive_force");
  nonneg(ps.rear_drive_force,  "rear_drive_force");
  nonneg(ps.front_regen_force, "front_regen_force");
  nonneg(ps.rear_regen_force,  "rear_regen_force");

  // Front+Rear == total (within tolerance) for each sign group
  const double dsum = ps.front_drive_force + ps.rear_drive_force;
  const double rsum = ps.front_regen_force + ps.rear_regen_force;

  if (std::abs(dsum - std::max(0.0, ps.drive_force)) > 1e-3 * (1.0 + std::abs(ps.drive_force)))
    fail(where, "drive split mismatch", dsum - ps.drive_force);
  if (std::abs(rsum - std::max(0.0, ps.regen_force)) > 1e-3 * (1.0 + std::abs(ps.regen_force)))
    fail(where, "regen split mismatch", rsum - ps.regen_force);

  // wheel_force = drive - regen (sign convention)
  const double wf_expected = ps.drive_force - ps.regen_force;
  if (std::abs(ps.wheel_force - wf_expected) > 1e-3 * (1.0 + std::abs(wf_expected)))
    fail(where, "wheel_force != drive - regen", ps.wheel_force - wf_expected);
}

inline void checkBrakes(const BrakeStatus& bs, const char* where)
{
  auto nonneg = [&](double v, const char* n){ if (v < -1e-6 || !finite(v)) fail(where, n, v); };
  nonneg(bs.command,              "brake.command");
  nonneg(bs.regen_command,        "brake.regen_command");
  nonneg(bs.regen_force,          "brake.regen_force");
  nonneg(bs.front_force,          "brake.front_force");
  nonneg(bs.rear_force,           "brake.rear_force");
  nonneg(bs.total_mechanical_force,"brake.total_mechanical_force");

  const double sum_mech = bs.front_force + bs.rear_force;
  if (std::abs(sum_mech - bs.total_mechanical_force) > 1e-3 * (1.0 + bs.total_mechanical_force))
    fail(where, "front+rear != total_mechanical", sum_mech - bs.total_mechanical_force);
}

// ------- wheels sanity -------
inline void checkWheels(const VehicleParam& p,
                        const WheelsInfo& w,
                        const VehicleState&,
                        const char* where)
{
  (void)p;
  auto ok = [](float v){ return std::isfinite(v) && std::fabs(v) < 1e5f; };
  if (!ok(w.lf_speed) || !ok(w.rf_speed) || !ok(w.lb_speed) || !ok(w.rb_speed))
    fail(where, "wheel speed invalid");
  if (!ok(w.steering))
    fail(where, "wheel steering invalid", w.steering);
}

inline void checkWheels(const WheelsInfo& w,
                        const char* where,
                        const Limits& L = Limits{})
{
  checkFinite(where, "lf_speed", w.lf_speed);
  checkFinite(where, "rf_speed", w.rf_speed);
  checkFinite(where, "lb_speed", w.lb_speed);
  checkFinite(where, "rb_speed", w.rb_speed);
  checkFinite(where, "steering", w.steering);
  if (std::abs(w.steering) > L.steering_abs_max + 0.05) fail(where, "steering too large", w.steering);
}

// ------- tire debug checks -------
struct TireDebugLike {
  double alpha_front{0.0};
  double alpha_rear{0.0};
  double fy_front{0.0};
  double fy_rear{0.0};
  double fx_total{0.0};
  double mu_lat{0.0};
  double speed{0.0};
  double steer{0.0};
};

inline void checkTireDebug(const VehicleParam& p,
                           double aF, double aR,
                           double fyF, double fyR,
                           double fxTot,
                           double mu_lat, double speed, double steer,
                           const char* where)
{
  (void)p;
  if (!finite(aF) || !finite(aR) || !finite(fyF) || !finite(fyR) ||
      !finite(fxTot) || !finite(mu_lat) || !finite(speed) || !finite(steer)) {
    fail(where, "tire debug contains !finite");
  }
  if (std::fabs(aF) > 1.7 || std::fabs(aR) > 1.7) fail(where, "slip angle > ~97deg");
  if (mu_lat < -1e-6) fail(where, "mu_lat negative", mu_lat);
}

inline void checkTireDebug(const TireDebugLike& td,
                           const char* where,
                           const Limits& L = Limits{})
{
  checkSlip(td.alpha_front, where, L);
  checkSlip(td.alpha_rear,  where, L);
  checkFinite(where, "fy_front", td.fy_front);
  checkFinite(where, "fy_rear",  td.fy_rear);
  checkFinite(where, "fx_total", td.fx_total);
  checkFinite(where, "mu_lat",   td.mu_lat);
  checkFinite(where, "speed",    td.speed);
  checkFinite(where, "steer",    td.steer);

  if (td.mu_lat < -1e-3 || td.mu_lat > 5.0) fail(where, "mu_lat out of range", td.mu_lat);
}

// ---------- convenience: bundle a full pass ----------

inline void checkFullStepPre(const VehicleParam& P,
                             const VehicleState& x,
                             const VehicleInput& u,
                             const char* where)
{
  Limits L;
  checkParam(P, L, where);
  checkState(x, where, L);
  checkInput(P, u, where, L);
}

inline void checkFullStepPost(const VehicleState& xNext,
                              const VehicleState& xDot,
                              const ForceSummary& F,
                              const PowertrainStatus& ps,
                              const BrakeStatus& bs,
                              const WheelsInfo&  wi,
                              const char* where)
{
  Limits L;
  checkState(xNext, where, L);
  checkDerivatives(xDot, where, L);
  checkForces(F, where, L);
  checkPowertrain(ps, where, L);
  checkBrakes(bs, where);
  checkWheels(wi, where, L);
}

}}} // namespace fsai::sim::sanity
