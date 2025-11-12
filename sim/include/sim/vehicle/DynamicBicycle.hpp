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

private:
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
