#pragma once

#include <cstdint>
#include <optional>

#include "adsdv_dbc.hpp"
#include "types.h"

namespace fsai::control::runtime {

struct Ai2VcuAdapterConfig {
  float front_motor_weight{0.5f};
  float rear_motor_weight{0.5f};
  float brake_front_bias{0.5f};
  float brake_rear_bias{0.5f};
  float max_speed_kph{120.0f};
  float motor_speed_max_rpm{4000.0f};
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
  };

  explicit Ai2VcuAdapter(const Ai2VcuAdapterConfig& config);

  Ai2VcuCommandSet Adapt(const fsai::types::ControlCmd& cmd,
                         const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
                         const AdapterTelemetry& telemetry = AdapterTelemetry{});

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
  void UpdateStatusFlags(const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
                         bool allow_motion);

  Ai2VcuAdapterConfig config_;
  fsai::sim::svcu::dbc::Ai2VcuStatus status_{};
  State state_{State::kIdle};
  bool handshake_level_{true};
};

}  // namespace fsai::control::runtime
