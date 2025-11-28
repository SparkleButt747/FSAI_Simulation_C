#pragma once

#include <string>
#include <vector>

#include "adsdv_dbc.hpp"
#include "types.h"
#include "sim/vehicle/VeloxVehicleDynamics.hpp"

namespace fsai::sim::app {

struct ControlSignalAdapterConfig {
  float max_abs_steer_rad{
      fsai::sim::svcu::dbc::kMaxSteerDeg * fsai::sim::svcu::dbc::kDegToRad};
  float max_throttle{1.0f};
  float max_brake{1.0f};
  float front_torque_fraction{0.0f};
  float rear_torque_fraction{1.0f};
  float front_axle_max_torque_nm{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};
  float rear_axle_max_torque_nm{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};
};

struct ControlSignalAdapterResult {
  fsai::types::ControlCmd clamped_cmd{};
  fsai::vehicle::VeloxVehicleDynamics::Command velox_command{};
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

// The adapter assumes AI control is continuously enabled; any disable condition
// must be detected and surfaced by the caller so that telemetry reports a hard
// failure instead of silently honoring manual inputs.
class ControlSignalAdapter {
 public:
  explicit ControlSignalAdapter(ControlSignalAdapterConfig config);

  ControlSignalAdapterResult Adapt(const fsai::types::ControlCmd& raw) const;

 private:
  ControlSignalAdapterConfig config_;

  [[nodiscard]] std::pair<float, float> torque_split() const;
  void log_messages(const std::vector<std::string>& messages,
                    bool is_error) const;
};

}  // namespace fsai::sim::app
