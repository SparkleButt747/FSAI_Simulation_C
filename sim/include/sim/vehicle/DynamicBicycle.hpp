#pragma once
#include "VehicleModel.hpp"
#include "EVPowertrain.hpp"
#include "BrakeController.hpp"

class DynamicBicycle : public VehicleModel {
public:
  using VehicleModel::VehicleModel;

  struct Forces {
    double Fx;   // total longitudinal [N]
    double FyF;  // front axle lateral [N]
    double FyR;  // rear axle lateral [N]
  };

  struct TireDebug {
    double alpha_front{0.0};
    double alpha_rear{0.0};
    double fy_front{0.0};
    double fy_rear{0.0};
    double fx_total{0.0};
    double mu_lat{0.0};
    double speed{0.0};
    double steer{0.0};
  };

  void updateState(VehicleState& state,
                   const VehicleInput& input,
                   double dt) override;

  const PowertrainStatus& lastPowertrainStatus() const { return last_pt_status_; }
  const BrakeStatus&      lastBrakeStatus()     const { return last_brake_status_; }
  const TireDebug&        lastTireDebug()       const { return tire_debug_; }

  static double calculateMagnitude(double x, double y) { return std::sqrt(x*x + y*y); }


  // Yaw governor: returns a corrective yaw acceleration term (add to xDot.rotation.z)
  double yawGovernor(double r,
                    double r_max,
                    double speed) const {
    if (r_max <= 0.0) return 0.0;
    // Normalize and saturate softly beyond limit
    const double u = r / r_max;          // desired in [-1,1]
    const double s = smoothSat01(u);     // softly clamp to [-1,1]
    const double err = r - s * r_max;    // how far beyond allowed band
    if (std::abs(err) < 1e-6) return 0.0;

    // Speed-aware damping gain: higher at speed
    const double base = 10.0;            // 1/s
    const double kspd = 0.35 * std::clamp(speed, 0.0, 40.0); // grows with speed
    const double k = base + kspd;
    // Apply acceleration toward the band (units rad/s^2)
    return -k * err;
  }

    // max yaw rate from lateral limit: |ay| <= mu*g and ay ≈ r*vx  -> |r| <= mu*g / max(vx, v_eps)
  static inline double rMaxFromFriction(double speed, double mu, double g) {
    const double v_eps = 0.8; // m/s, don't explode at standstill
    return ayLimit(mu, g) / std::max(speed, v_eps);
  }

private:
  // --- Spin-prevention helpers (header-only) ---
  static inline double ayLimit(double mu, double g) {
    return std::max(0.5, mu * g); // floor so limit isn't absurdly tiny
  }

  // Smooth odd saturator in [-1,1] with C1 continuity
  static inline double smoothSat01(double x) {
    // clamp to [-1.5,1.5] then ease in-out
    const double xc = std::clamp(x, -1.5, 1.5);
    const double a = std::abs(xc);
    const double y = (a < 1.0) ? (a*a*(3.0 - 2.0*a)) : 1.0;
    return std::copysign(y, xc);
  }

  // Steer soft limiter approaching yaw limit
  double steerSoftLimit(double delta_cmd,
                        double r, double r_max) const {
    if (r_max <= 0.0) return 0.0;
    const double u = r / r_max;              // normalized yaw usage
    const double w = std::abs(u);            // 0..∞
    // Gain taper: 1 at small w, falls as we exceed ~0.7, clamps hard past 1
    const double w0 = 0.70;
    const double k  = 4.0;                   // steepness
    double gain = 1.0;
    if (w > w0) {
      const double t = std::clamp((w - w0) / std::max(1e-6, 1.0 - w0), 0.0, 1.0);
      gain = 1.0 / (1.0 + k * t * t);
    }
    return delta_cmd * gain;
  }

  // Reduce wheel drive/brake when spinning to free up lateral capacity
  double longForceSpinScale(double r, double r_max) const {
    if (r_max <= 0.0) return 0.5;
    const double w = std::abs(r) / r_max;
    if (w <= 0.8) return 1.0;
    if (w >= 1.3) return 0.25;
    // smooth fall between 0.8..1.3
    const double t = (w - 0.8) / 0.5; // 0..1
    return 1.0 - 0.75 * (t * t * (3.0 - 2.0 * t)); // Hermite
  }

  // Actuator effective commands (lagged)
  mutable double thr_eff_{0.0};
  mutable double brk_eff_{0.0};

  // Runtime subsystems
  mutable EVMotorPowertrain pt_;
  mutable BrakeController   br_;
  mutable PowertrainStatus  last_pt_status_{};
  mutable BrakeStatus       last_brake_status_{};
  mutable bool              systems_configured_{false};

  // Slip relaxation states
  mutable double alpha_front_rel_{0.0};
  mutable double alpha_rear_rel_{0.0};

  // Tire debug telemetry
  mutable TireDebug tire_debug_{};

  Forces computeForces(const VehicleState& state,
                       const VehicleInput& input,
                       double dt) const;

  static inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
  static inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
};
