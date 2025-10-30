#pragma once

#include <cstdint>
#include <optional>

#include <io/can/adsdv_dbc.hpp>
#include <io/can/ai2vcu_types.hpp>
#include <common/types.h>

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

  fsai::io::can::Ai2VcuCommandSet Adapt(
      const fsai::types::ControlCmd& cmd,
      const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback);
  fsai::io::can::Ai2VcuCommandSet Adapt(
      const fsai::types::ControlCmd& cmd,
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
  void UpdateStatusFlags(const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
                         bool allow_motion);

  Ai2VcuAdapterConfig config_;
  fsai::sim::svcu::dbc::Ai2VcuStatus status_{};
  State state_{State::kIdle};
  bool handshake_level_{true};
};

}  // namespace fsai::control::runtime
