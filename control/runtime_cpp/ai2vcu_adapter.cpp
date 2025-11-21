#include "ai2vcu_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsai::control::runtime {
namespace {
constexpr float kEpsilon = 1e-6f;

uint8_t MissionTypeToId(fsai::sim::MissionType type) {
  switch (type) {
    case fsai::sim::MissionType::kAcceleration:
      return 1u;
    case fsai::sim::MissionType::kSkidpad:
      return 2u;
    case fsai::sim::MissionType::kAutocross:
      return 3u;
    case fsai::sim::MissionType::kTrackdrive:
      return 4u;
    default:
      return 0u;
  }
}
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

  mission_id_ = ComputeMissionId(config_.mission_descriptor);
  if (mission_id_.has_value()) {
    status_.mission_id = *mission_id_;
  }
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
  if (telemetry.mission_laps_completed.has_value()) {
    status_.cones_count_actual = static_cast<uint8_t>(
        std::min<uint16_t>(*telemetry.mission_laps_completed, std::numeric_limits<uint8_t>::max()));
  }
  if (telemetry.mission_laps_target.has_value()) {
    status_.cones_count_all = *telemetry.mission_laps_target;
  }

  UpdateMissionId(telemetry);
  UpdateMissionStatus(telemetry);
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

Ai2VcuCommandSet Ai2VcuAdapter::Adapt(
    const fsai::types::ControlCmd& cmd,
    const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback) {
  return Adapt(cmd, feedback, AdapterTelemetry{});
}

Ai2VcuCommandSet Ai2VcuAdapter::Adapt(
    const fsai::types::ControlCmd& cmd,
    const fsai::sim::svcu::dbc::Vcu2AiStatus& feedback,
    const AdapterTelemetry& telemetry) {
  ToggleHandshake();
  UpdateTelemetry(telemetry);
  UpdateState(feedback);

  const bool request_forward = mission_running_ && state_ != State::kSafeStop && !mission_finished_;
  const bool allow_motion = request_forward && feedback.go_signal && !mission_finished_;

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
  UpdateStatusFlags(allow_motion, request_forward);

  Ai2VcuCommandSet out{};
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

uint8_t Ai2VcuAdapter::ComputeMissionId(
    const std::optional<fsai::sim::MissionDescriptor>& descriptor) {
  if (!descriptor.has_value()) {
    return 0u;
  }
  return MissionTypeToId(descriptor->type);
}

void Ai2VcuAdapter::UpdateMissionId(const AdapterTelemetry& telemetry) {
  if (telemetry.mission_id.has_value()) {
    mission_id_ = static_cast<uint8_t>(
        std::min<uint8_t>(*telemetry.mission_id, static_cast<uint8_t>(15u)));
  } else if (!mission_id_.has_value()) {
    const uint8_t computed = ComputeMissionId(config_.mission_descriptor);
    if (computed != 0u) {
      mission_id_ = computed;
    }
  }

  if (mission_id_.has_value()) {
    status_.mission_id = *mission_id_;
  } else {
    status_.mission_id = 0u;
  }
}

void Ai2VcuAdapter::UpdateMissionStatus(const AdapterTelemetry& telemetry) {
  if (telemetry.mission_selected.has_value()) {
    mission_selected_ = *telemetry.mission_selected;
  }
  if (telemetry.mission_running.has_value()) {
    mission_running_ = *telemetry.mission_running;
  }
  if (telemetry.mission_finished.has_value()) {
    mission_finished_ = *telemetry.mission_finished;
  }

  if (!mission_selected_) {
    mission_running_ = false;
    mission_finished_ = false;
  }
  if (mission_finished_) {
    mission_running_ = false;
  }

  if (mission_finished_) {
    mission_status_ = fsai::sim::svcu::dbc::MissionStatus::kFinished;
  } else if (mission_running_) {
    mission_status_ = fsai::sim::svcu::dbc::MissionStatus::kRunning;
  } else if (mission_selected_) {
    mission_status_ = fsai::sim::svcu::dbc::MissionStatus::kSelected;
  } else {
    mission_status_ = fsai::sim::svcu::dbc::MissionStatus::kNotSelected;
  }

  status_.mission_status = mission_status_;
  status_.mission_complete = mission_finished_;
}

void Ai2VcuAdapter::UpdateStatusFlags(bool allow_motion, bool request_forward) {
  if (state_ == State::kSafeStop) {
    mission_status_ = fsai::sim::svcu::dbc::MissionStatus::kFinished;
    mission_finished_ = true;
    status_.mission_complete = true;
  }

  status_.mission_status = mission_status_;

  status_.estop_request = (state_ == State::kSafeStop);
  if (!allow_motion) {
    status_.veh_speed_demand_kph = 0.0f;
  }

  const bool neutral = !request_forward || state_ == State::kSafeStop;
  status_.direction_request =
      neutral ? fsai::sim::svcu::dbc::DirectionRequest::kNeutral
              : fsai::sim::svcu::dbc::DirectionRequest::kForward;
}

}  // namespace fsai::control::runtime
