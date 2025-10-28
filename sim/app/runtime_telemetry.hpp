#pragma once

#include <optional>
#include <string>

#include "ai2vcu_adapter.hpp"
#include "can_iface.hpp"
#include "types.h"

namespace fsai::sim::app {

struct RuntimeTelemetry {
  struct PhysicsData {
    double simulation_time_s{0.0};
    float vehicle_speed_mps{0.0f};
    float vehicle_speed_kph{0.0f};
    float steering_deg{0.0f};
  } physics;

  struct CanData {
    fsai::sim::svcu::dbc::Vcu2AiStatus status{};
    fsai::sim::svcu::dbc::Vcu2AiSteer steer{};
    fsai::sim::svcu::dbc::Vcu2AiDrive front_drive{};
    fsai::sim::svcu::dbc::Vcu2AiDrive rear_drive{};
    fsai::sim::svcu::dbc::Vcu2AiBrake brake{};
    fsai::sim::svcu::dbc::Vcu2LogDynamics1 dynamics{};
    std::optional<fsai::types::VehicleState> vehicle_state{};
    std::optional<fsai::control::runtime::ImuSample> imu{};
    std::optional<fsai::control::runtime::GpsSample> gps{};
  } can;

  struct ControlData {
    fsai::types::ControlCmd control_cmd{};
    float applied_throttle{0.0f};
    float applied_brake{0.0f};
    float applied_steer_rad{0.0f};
    bool has_last_command{false};
    std::optional<fsai::control::runtime::Ai2VcuCommandSet> last_command{};
  } control;

  struct ModeData {
    bool use_controller{false};
    std::string runtime_mode;
  } mode;
};

}  // namespace fsai::sim::app

