#pragma once

#include <cstdint>
#include <optional>

#include "adsdv_dbc.hpp"
#include "types.h"
#include "sim/mission_descriptor.hpp"

namespace fsai::control::runtime {

struct Ai2VcuAdapterConfig {
  float front_torque_fraction{0.0f};
  float rear_torque_fraction{1.0f};
  float front_axle_max_torque_nm{0.0f};
  float rear_axle_max_torque_nm{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};
  float brake_front_bias{0.5f};
  float brake_rear_bias{0.5f};
  float max_speed_kph{120.0f};
  float motor_speed_max_rpm{4000.0f};
  std::optional<fsai::sim::MissionDescriptor> mission_descriptor;
};

struct Ai2VcuCommandSet {
  fsai::sim::svcu::dbc::Ai2VcuStatus status;
  fsai::sim::svcu::dbc::Ai2VcuSteer steer;
  fsai::sim::svcu::dbc::Ai2VcuDrive front_drive;
  fsai::sim::svcu::dbc::Ai2VcuDrive rear_drive;
  fsai::sim::svcu::dbc::Ai2VcuBrake brake;
  float throttle_clamped{0.0f};
  float brake_clamped{0.0f};
};

class Ai2VcuAdapter {
 public:
  struct AdapterTelemetry {
    float measured_speed_mps{0.0f};
    std::optional<uint8_t> lap_counter;
    std::optional<uint8_t> cones_count_actual;
    std::optional<uint16_t> cones_count_all;
    std::optional<uint16_t> mission_laps_completed;
    std::optional<uint16_t> mission_laps_target;
    std::optional<bool> mission_selected;
    std::optional<bool> mission_running;
    std::optional<bool> mission_finished;
    std::optional<uint8_t> mission_id;
  };

  explicit Ai2VcuAdapter(const Ai2VcuAdapterConfig& config);

  static Ai2VcuAdapterConfig SanitizeConfig(const Ai2VcuAdapterConfig& config);
  const Ai2VcuAdapterConfig& config() const { return config_; }

  Ai2VcuCommandSet Adapt(const fsai::types::ControlCmd& cmd,
                         const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback);
  Ai2VcuCommandSet Adapt(const fsai::types::ControlCmd& cmd,
                         const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
                         const AdapterTelemetry& telemetry);

 private:
  enum class State {
    kIdle,
    kArmed,
    kRunning,
    kSafeStop,
  };

  void ToggleHandshake();
  void UpdateTelemetry(const AdapterTelemetry& telemetry);
  void UpdateState(const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback);
  bool ShouldEnterSafeStop(const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback) const;
  void UpdateStatusFlags(bool allow_motion, bool request_forward);
  void UpdateMissionStatus(const AdapterTelemetry& telemetry);
  void UpdateMissionId(const AdapterTelemetry& telemetry);
  static uint8_t ComputeMissionId(const std::optional<fsai::sim::MissionDescriptor>& descriptor);

  Ai2VcuAdapterConfig config_;
  fsai::sim::svcu::dbc::Ai2VcuStatus status_{};
  State state_{State::kIdle};
  bool handshake_level_{true};
  std::optional<uint8_t> mission_id_{};
  fsai::sim::svcu::dbc::MissionStatus mission_status_{fsai::sim::svcu::dbc::MissionStatus::kNotSelected};
  bool mission_selected_{false};
  bool mission_running_{false};
  bool mission_finished_{false};
};

}  // namespace fsai::control::runtime
