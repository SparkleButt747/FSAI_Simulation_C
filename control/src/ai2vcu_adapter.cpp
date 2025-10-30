#include <control/ai2vcu_adapter.hpp>

#include <algorithm>
#include <cmath>

namespace fsai::control::runtime {
namespace {
constexpr float kEpsilon = 1e-6f;
}

Ai2VcuAdapter::Ai2VcuAdapter(const Ai2VcuAdapterConfig& config)
    : config_(config) {
  float front_fraction = std::max(0.0f, config_.front_torque_fraction);
  float rear_fraction = std::max(0.0f, config_.rear_torque_fraction);
  if (front_fraction <= kEpsilon && rear_fraction <= kEpsilon) {
    rear_fraction = 1.0f;
  }
  const float fraction_sum = std::max(kEpsilon, front_fraction + rear_fraction);
  config_.front_torque_fraction = std::clamp(front_fraction / fraction_sum, 0.0f, 1.0f);
  config_.rear_torque_fraction = std::clamp(rear_fraction / fraction_sum, 0.0f, 1.0f);

  config_.front_axle_max_torque_nm =
      std::clamp(config_.front_axle_max_torque_nm, 0.0f,
                 fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
  config_.rear_axle_max_torque_nm =
      std::clamp(config_.rear_axle_max_torque_nm <= 0.0f
                     ? fsai::sim::svcu::dbc::kMaxAxleTorqueNm
                     : config_.rear_axle_max_torque_nm,
                 0.0f, fsai::sim::svcu::dbc::kMaxAxleTorqueNm);

  const float brake_sum = std::max(kEpsilon, config_.brake_front_bias + config_.brake_rear_bias);
  config_.brake_front_bias = std::clamp(config_.brake_front_bias / brake_sum, 0.0f, 1.0f);
  config_.brake_rear_bias = std::clamp(1.0f - config_.brake_front_bias, 0.0f, 1.0f);

  status_.handshake = handshake_level_;
  status_.mission_status = fsai::sim::svcu::dbc::MissionStatus::kNotSelected;
  status_.direction_request = fsai::sim::svcu::dbc::DirectionRequest::kNeutral;
  status_.veh_speed_actual_kph = 0.0f;
  status_.veh_speed_demand_kph = 0.0f;
}

void Ai2VcuAdapter::ToggleHandshake() {
  handshake_level_ = !handshake_level_;
  status_.handshake = handshake_level_;
}

void Ai2VcuAdapter::UpdateTelemetry(const AdapterTelemetry& telemetry) {
  status_.veh_speed_actual_kph =
      std::clamp(telemetry.measured_speed_mps * 3.6f, 0.0f, 255.0f);
  if (telemetry.lap_counter.has_value()) {
    status_.lap_counter = static_cast<uint8_t>(*telemetry.lap_counter & 0x0Fu);
  }
  if (telemetry.cones_count_actual.has_value()) {
    status_.cones_count_actual = *telemetry.cones_count_actual;
  }
  if (telemetry.cones_count_all.has_value()) {
    status_.cones_count_all = *telemetry.cones_count_all;
  }
}

bool Ai2VcuAdapter::ShouldEnterSafeStop(
    const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback) const {
  if (feedback.shutdown_request) {
    return true;
  }
  if (feedback.as_state == fsai::sim::svcu::dbc::AsState::kEmergencyBrake ||
      feedback.as_state == fsai::sim::svcu::dbc::AsState::kFinished) {
    return true;
  }
  return feedback.fault || feedback.ebs_fault || feedback.brake_plausibility_fault ||
         feedback.bms_fault || feedback.autonomous_braking_fault ||
         feedback.mission_status_fault || feedback.hvil_open_fault ||
         feedback.hvil_short_fault || feedback.ai_comms_lost ||
         feedback.ai_estop_request;
}

void Ai2VcuAdapter::UpdateState(const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback) {
  if (state_ == State::kSafeStop) {
    return;
  }
  if (ShouldEnterSafeStop(feedback)) {
    state_ = State::kSafeStop;
    return;
  }

  switch (feedback.as_state) {
    case fsai::sim::svcu::dbc::AsState::kDriving:
      state_ = feedback.go_signal ? State::kRunning : State::kArmed;
      break;
    case fsai::sim::svcu::dbc::AsState::kReady:
      state_ = feedback.go_signal ? State::kRunning : State::kArmed;
      break;
    case fsai::sim::svcu::dbc::AsState::kOff:
    case fsai::sim::svcu::dbc::AsState::kR2d:
      state_ = State::kIdle;
      break;
    default:
      state_ = State::kIdle;
      break;
  }
}

void Ai2VcuAdapter::UpdateStatusFlags(
    const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
    bool allow_motion) {
  switch (state_) {
    case State::kIdle:
      status_.mission_status = fsai::sim::svcu::dbc::MissionStatus::kNotSelected;
      break;
    case State::kArmed:
      status_.mission_status = fsai::sim::svcu::dbc::MissionStatus::kSelected;
      break;
    case State::kRunning:
      status_.mission_status = allow_motion
                                   ? fsai::sim::svcu::dbc::MissionStatus::kRunning
                                   : fsai::sim::svcu::dbc::MissionStatus::kSelected;
      break;
    case State::kSafeStop:
      status_.mission_status = fsai::sim::svcu::dbc::MissionStatus::kFinished;
      break;
  }

  status_.estop_request = !allow_motion;
  status_.direction_request =
      allow_motion ? fsai::sim::svcu::dbc::DirectionRequest::kForward
                   : fsai::sim::svcu::dbc::DirectionRequest::kNeutral;
}

fsai::io::can::Ai2VcuCommandSet Ai2VcuAdapter::Adapt(
    const fsai::types::ControlCmd& cmd,
    const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback) {
  return Adapt(cmd, feedback, AdapterTelemetry{});
}

fsai::io::can::Ai2VcuCommandSet Ai2VcuAdapter::Adapt(
    const fsai::types::ControlCmd& cmd,
    const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
    const AdapterTelemetry& telemetry) {
  ToggleHandshake();
  UpdateTelemetry(telemetry);
  UpdateState(feedback);

  const bool allow_motion = state_ == State::kRunning && feedback.go_signal;

  float requested_throttle =
      allow_motion ? std::clamp(cmd.throttle, 0.0f, 1.0f) : 0.0f;
  float requested_brake =
      allow_motion ? std::clamp(cmd.brake, 0.0f, 1.0f) : 0.0f;

  // The simulated VCU expects mutually exclusive throttle/brake requests.  When
  // both arrive non-zero we project them onto the 1D acceleration axis so that
  // only the dominant command is applied.
  const float net_accel = requested_throttle - requested_brake;
  const float throttle = std::clamp(net_accel, 0.0f, 1.0f);
  const float brake = std::clamp(-net_accel, 0.0f, 1.0f);

  status_.veh_speed_demand_kph = allow_motion
                                     ? std::clamp(config_.max_speed_kph * throttle, 0.0f, 255.0f)
                                     : 0.0f;
  UpdateStatusFlags(feedback, allow_motion);

  fsai::io::can::Ai2VcuCommandSet out{};
  out.throttle_clamped = throttle;
  out.brake_clamped = brake;

  if (allow_motion) {
    auto quantize = [](float value, float step) {
      if (step <= 0.0f) {
        return value;
      }
      return std::round(value / step) * step;
    };

    const float steer_deg = cmd.steer_rad * fsai::sim::svcu::dbc::kRadToDeg;
    const float steer_deg_clamped = std::clamp(steer_deg, -fsai::sim::svcu::dbc::kMaxSteerDeg,
                                               fsai::sim::svcu::dbc::kMaxSteerDeg);
    out.steer.steer_deg = std::clamp(quantize(steer_deg_clamped, 0.1f),
                                     -fsai::sim::svcu::dbc::kMaxSteerDeg,
                                     fsai::sim::svcu::dbc::kMaxSteerDeg);

    const float front_torque_target =
        throttle * config_.front_torque_fraction * config_.front_axle_max_torque_nm;
    const float rear_torque_target =
        throttle * config_.rear_torque_fraction * config_.rear_axle_max_torque_nm;
    out.front_drive.axle_torque_request_nm = std::clamp(
        front_torque_target, 0.0f, config_.front_axle_max_torque_nm);
    out.rear_drive.axle_torque_request_nm = std::clamp(
        rear_torque_target, 0.0f, config_.rear_axle_max_torque_nm);
    out.front_drive.motor_speed_max_rpm = config_.motor_speed_max_rpm;
    out.rear_drive.motor_speed_max_rpm = config_.motor_speed_max_rpm;

    const float brake_pct = brake * fsai::sim::svcu::dbc::kMaxBrakePercent;
    const float front_brake_clamped = std::clamp(brake_pct * config_.brake_front_bias, 0.0f,
                                                 fsai::sim::svcu::dbc::kMaxBrakePercent);
    const float rear_brake_clamped = std::clamp(brake_pct * config_.brake_rear_bias, 0.0f,
                                                fsai::sim::svcu::dbc::kMaxBrakePercent);
    out.brake.front_pct = std::clamp(quantize(front_brake_clamped, 0.5f), 0.0f,
                                     fsai::sim::svcu::dbc::kMaxBrakePercent);
    out.brake.rear_pct = std::clamp(quantize(rear_brake_clamped, 0.5f), 0.0f,
                                    fsai::sim::svcu::dbc::kMaxBrakePercent);
  } else {
    out.steer.steer_deg = 0.0f;
    out.front_drive.axle_torque_request_nm = 0.0f;
    out.rear_drive.axle_torque_request_nm = 0.0f;
    out.front_drive.motor_speed_max_rpm = 0.0f;
    out.rear_drive.motor_speed_max_rpm = 0.0f;
    out.brake.front_pct = 0.0f;
    out.brake.rear_pct = 0.0f;
  }

  out.status = status_;
  return out;
}

}  // namespace fsai::control::runtime
