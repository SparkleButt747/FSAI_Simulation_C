#include "control_signal_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "../include/logging.hpp"

namespace fsai::sim::app {

ControlSignalAdapter::ControlSignalAdapter(ControlSignalAdapterConfig config)
    : config_(config) {
  if (config_.max_abs_steer_rad <= 0.0f) {
    config_.max_abs_steer_rad =
        fsai::sim::svcu::dbc::kMaxSteerDeg * fsai::sim::svcu::dbc::kDegToRad;
  }
  config_.front_torque_fraction = std::max(0.0f, config_.front_torque_fraction);
  config_.rear_torque_fraction = std::max(0.0f, config_.rear_torque_fraction);
  config_.front_axle_max_torque_nm = std::clamp(
      config_.front_axle_max_torque_nm, 0.0f, fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
  config_.rear_axle_max_torque_nm = std::clamp(
      config_.rear_axle_max_torque_nm, 0.0f, fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
}

namespace {
float clamp_and_note(float value, float lo, float hi, const char* name,
                     std::vector<std::string>& errors,
                     std::vector<std::string>& warnings) {
  if (!std::isfinite(value)) {
    errors.emplace_back(std::string(name) + " is not finite; resetting to 0");
    return 0.0f;
  }
  if (value < lo || value > hi) {
    errors.emplace_back(std::string(name) + " clamped into [" + std::to_string(lo) +
                        ", " + std::to_string(hi) + "]");
  }
  return std::clamp(value, lo, hi);
}
}  // namespace

// The AI command stream is assumed to stay enabled; disable conditions must be
// handled before calling Adapt so that failures are raised explicitly.
ControlSignalAdapterResult ControlSignalAdapter::Adapt(
    const fsai::types::ControlCmd& raw) const {
  ControlSignalAdapterResult result{};
  result.clamped_cmd = raw;
  result.velox_command.steer_rad = raw.steer_rad;

  auto [front_fraction, rear_fraction] = torque_split();

  result.clamped_cmd.steer_rad =
      clamp_and_note(raw.steer_rad, -config_.max_abs_steer_rad, config_.max_abs_steer_rad,
                     "steer_rad", result.errors, result.warnings);

  const float throttle_raw =
      clamp_and_note(raw.throttle, 0.0f, config_.max_throttle, "throttle",
                     result.errors, result.warnings);
  const float brake_raw =
      clamp_and_note(raw.brake, 0.0f, config_.max_brake, "brake",
                     result.errors, result.warnings);

  // Project simultaneous throttle+brake onto one axis to avoid conflicting requests.
  const float net = throttle_raw - brake_raw;
  const float throttle = std::clamp(net, 0.0f, config_.max_throttle);
  const float brake = std::clamp(-net, 0.0f, config_.max_brake);
  result.clamped_cmd.throttle = throttle;
  result.clamped_cmd.brake = brake;

  result.velox_command.steer_rad = result.clamped_cmd.steer_rad;

  const float front_drive_torque =
      result.clamped_cmd.throttle * front_fraction * config_.front_axle_max_torque_nm;
  const float rear_drive_torque =
      result.clamped_cmd.throttle * rear_fraction * config_.rear_axle_max_torque_nm;
  const float front_brake_torque =
      result.clamped_cmd.brake * front_fraction * config_.front_axle_max_torque_nm;
  const float rear_brake_torque =
      result.clamped_cmd.brake * rear_fraction * config_.rear_axle_max_torque_nm;

  const float front_net_torque = front_drive_torque - front_brake_torque;
  const float rear_net_torque = rear_drive_torque - rear_brake_torque;

  const float front_max_torque = config_.front_axle_max_torque_nm;
  const float rear_max_torque = config_.rear_axle_max_torque_nm;
  result.velox_command.front_axle_torque_nm = std::clamp(front_net_torque, -front_max_torque, front_max_torque);
  result.velox_command.rear_axle_torque_nm = std::clamp(rear_net_torque, -rear_max_torque, rear_max_torque);

  constexpr float kTorqueThreshold = 1e-6f;
  const bool torque_mode_enabled =
      front_max_torque > kTorqueThreshold || rear_max_torque > kTorqueThreshold;
  if (torque_mode_enabled) {
    result.velox_command.throttle = 0.0f;
    result.velox_command.brake = 0.0f;
  } else {
    result.velox_command.throttle = result.clamped_cmd.throttle;
    result.velox_command.brake = result.clamped_cmd.brake;
  }

  log_messages(result.errors, /*is_error=*/true);
  log_messages(result.warnings, /*is_error=*/false);

  return result;
}

std::pair<float, float> ControlSignalAdapter::torque_split() const {
  float front = std::max(0.0f, config_.front_torque_fraction);
  float rear = std::max(0.0f, config_.rear_torque_fraction);
  const float sum = front + rear;
  if (sum <= 1e-6f) {
    return {0.5f, 0.5f};
  }
  return {front / sum, rear / sum};
}

void ControlSignalAdapter::log_messages(const std::vector<std::string>& messages,
                                        bool is_error) const {
  for (const auto& msg : messages) {
    if (is_error) {
      fsai::sim::log::LogError(msg);
    } else {
      fsai::sim::log::LogWarning(msg);
    }
  }
}

}  // namespace fsai::sim::app
