#pragma once

#include <array>
#include <limits>
#include <optional>
#include <string>

#include "ai2vcu_adapter.hpp"
#include "can_iface.hpp"
#include "types.h"

namespace fsai::sim::app {

struct RuntimeTelemetry {
  template <typename T>
  struct TimedSample {
    T value{};
    double age_s{std::numeric_limits<double>::infinity()};
    bool valid{false};
  };

  struct PhysicsData {
    double simulation_time_s{0.0};
    float vehicle_speed_mps{0.0f};
    float vehicle_speed_kph{0.0f};
    float steering_deg{0.0f};
  } physics;

  struct PoseData {
    float position_x_m{0.0f};
    float position_y_m{0.0f};
    float position_z_m{0.0f};
    float yaw_deg{0.0f};
  } pose;

  struct WheelData {
    std::array<float, 4> rpm{0.0f, 0.0f, 0.0f, 0.0f};
  } wheels;

  struct DriveData {
    float front_axle_torque_nm{0.0f};
    float rear_axle_torque_nm{0.0f};
    float front_drive_force_n{0.0f};
    float rear_drive_force_n{0.0f};
    float front_net_force_n{0.0f};
    float rear_net_force_n{0.0f};
  } drive;

  struct BrakeData {
    float front_force_n{0.0f};
    float rear_force_n{0.0f};
    float front_pct{0.0f};
    float rear_pct{0.0f};
  } brake;

  struct AccelerationData {
    float longitudinal_mps2{0.0f};
    float lateral_mps2{0.0f};
    float vertical_mps2{0.0f};
    float yaw_rate_degps{0.0f};
  } acceleration;

  struct LapData {
    double current_lap_time_s{0.0};
    double total_distance_m{0.0};
    int completed_laps{0};
  } lap;

  struct CanData {
    fsai::control::runtime::CanIface::Mode mode{fsai::control::runtime::CanIface::Mode::kSimulation};
    std::string endpoint;
    double last_heartbeat_age_s{std::numeric_limits<double>::infinity()};
    TimedSample<fsai::sim::svcu::dbc::Vcu2AiStatus> status{};
    TimedSample<fsai::sim::svcu::dbc::Vcu2AiSteer> steer{};
    TimedSample<fsai::sim::svcu::dbc::Vcu2AiDrive> front_drive{};
    TimedSample<fsai::sim::svcu::dbc::Vcu2AiDrive> rear_drive{};
    TimedSample<fsai::sim::svcu::dbc::Vcu2AiBrake> brake{};
    TimedSample<fsai::sim::svcu::dbc::Vcu2AiSpeeds> speeds{};
    TimedSample<fsai::sim::svcu::dbc::Vcu2LogDynamics1> dynamics{};
    std::optional<fsai::types::VehicleState> vehicle_state{};
    std::optional<fsai::control::runtime::ImuSample> imu{};
    std::optional<fsai::control::runtime::GpsSample> gps{};
  } can;

  struct ControlData {
    fsai::types::ControlCmd control_cmd{};
    float applied_throttle{0.0f};
    float applied_brake{0.0f};
    float applied_steer_rad{0.0f};
    TimedSample<fsai::types::ControlCmd> ai_command{};
    bool ai_command_enabled{false};
    bool ai_command_applied{false};
    bool has_last_command{false};
    std::optional<fsai::control::runtime::Ai2VcuCommandSet> last_command{};
  } control;

  struct ModeData {
    bool use_controller{false};
    std::string runtime_mode;
  } mode;
};

}  // namespace fsai::sim::app

