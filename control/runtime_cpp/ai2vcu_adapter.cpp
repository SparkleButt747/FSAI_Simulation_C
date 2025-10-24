#include "ai2vcu_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::control::runtime {
namespace {
constexpr float kEpsilon = 1e-6f;
}

Ai2VcuAdapter::Ai2VcuAdapter(const Ai2VcuAdapterConfig& config)
    : config_(config) {
  const float motor_sum = std::max(kEpsilon, config_.front_motor_weight + config_.rear_motor_weight);
  config_.front_motor_weight = std::clamp(config_.front_motor_weight / motor_sum, 0.0f, 1.0f);
  config_.rear_motor_weight = std::clamp(config_.rear_motor_weight / motor_sum, 0.0f, 1.0f);

  const float brake_sum = std::max(kEpsilon, config_.brake_front_bias + config_.brake_rear_bias);
  config_.brake_front_bias = std::clamp(config_.brake_front_bias / brake_sum, 0.0f, 1.0f);
  config_.brake_rear_bias = std::clamp(1.0f - config_.brake_front_bias, 0.0f, 1.0f);

  status_.handshake = true;
  status_.mission_status = fsai::sim::svcu::dbc::MissionStatus::kSelected;
  status_.direction_request = fsai::sim::svcu::dbc::DirectionRequest::kForward;
  status_.veh_speed_actual_kph = 0.0f;
  status_.veh_speed_demand_kph = 0.0f;
}

Ai2VcuCommandSet Ai2VcuAdapter::Adapt(const fsai::types::ControlCmd& cmd,
                                      float measured_speed_mps,
                                      bool mission_running,
                                      uint8_t lap_counter) {
  Ai2VcuCommandSet out{};

  const float throttle = std::clamp(cmd.throttle, 0.0f, 1.0f);
  const float brake = std::clamp(cmd.brake, 0.0f, 1.0f);
  out.throttle_clamped = throttle;
  out.brake_clamped = brake;

  status_.mission_status = mission_running ? fsai::sim::svcu::dbc::MissionStatus::kRunning
                                           : fsai::sim::svcu::dbc::MissionStatus::kSelected;
  status_.estop_request = false;
  status_.lap_counter = static_cast<uint8_t>(lap_counter & 0x0Fu);
  status_.veh_speed_actual_kph = std::clamp(measured_speed_mps * 3.6f, 0.0f, 255.0f);
  status_.veh_speed_demand_kph = std::clamp(config_.max_speed_kph * throttle, 0.0f, 255.0f);
  out.status = status_;

  const float steer_deg = std::clamp(cmd.steer_rad * fsai::sim::svcu::dbc::kRadToDeg,
                                     -fsai::sim::svcu::dbc::kMaxSteerDeg,
                                     fsai::sim::svcu::dbc::kMaxSteerDeg);
  out.steer.steer_deg = steer_deg;

  const float total_torque_nm = throttle * 2.0f * fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
  out.front_drive.axle_torque_request_nm =
      std::clamp(total_torque_nm * config_.front_motor_weight, 0.0f,
                 fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
  out.rear_drive.axle_torque_request_nm =
      std::clamp(total_torque_nm * config_.rear_motor_weight, 0.0f,
                 fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
  out.front_drive.motor_speed_max_rpm = config_.motor_speed_max_rpm;
  out.rear_drive.motor_speed_max_rpm = config_.motor_speed_max_rpm;

  const float brake_pct = brake * fsai::sim::svcu::dbc::kMaxBrakePercent;
  out.brake.front_pct = std::clamp(brake_pct * config_.brake_front_bias, 0.0f,
                                   fsai::sim::svcu::dbc::kMaxBrakePercent);
  out.brake.rear_pct = std::clamp(brake_pct * config_.brake_rear_bias, 0.0f,
                                  fsai::sim::svcu::dbc::kMaxBrakePercent);

  return out;
}

}  // namespace fsai::control::runtime
