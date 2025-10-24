#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "adsdv_dbc.hpp"
#include "link.hpp"
#include "can_link.hpp"
#include "VehicleParam.hpp"
#include "svcu_runner.hpp"

namespace {
struct SanitizedCommand {
  float steer_rad{0.0f};
  float throttle{0.0f};
  float brake{1.0f};
  float front_torque_request_nm{0.0f};
  float rear_torque_request_nm{0.0f};
  float front_brake_req_pct{fsai::sim::svcu::dbc::kMaxBrakePercent};
  float rear_brake_req_pct{fsai::sim::svcu::dbc::kMaxBrakePercent};
  bool enabled{false};
};

class CommandAggregator {
 public:
  explicit CommandAggregator(const VehicleParam& params)
      : accel_min_(static_cast<float>(params.input_ranges.acc.min)),
        accel_max_(static_cast<float>(params.input_ranges.acc.max)),
        steer_min_(static_cast<float>(params.input_ranges.delta.min)),
        steer_max_(static_cast<float>(params.input_ranges.delta.max)),
        brake_front_bias_(static_cast<float>(params.brakes.front_bias)),
        brake_rear_bias_(static_cast<float>(params.brakes.rear_bias)),
        brake_max_force_(static_cast<float>(params.brakes.max_force)) {
    const float bias_sum = brake_front_bias_ + brake_rear_bias_;
    if (bias_sum <= 0.0f) {
      brake_front_bias_ = 0.5f;
      brake_rear_bias_ = 0.5f;
    } else {
      brake_front_bias_ = std::clamp(brake_front_bias_ / bias_sum, 0.0f, 1.0f);
      brake_rear_bias_ = 1.0f - brake_front_bias_;
    }
    max_throttle_ = accel_max_ > 0.0f ? std::min(accel_max_, 1.0f) : 1.0f;
    max_brake_ = accel_min_ < 0.0f ? std::min(-accel_min_, 1.0f) : 1.0f;
    sanitized_.brake = max_brake_;
    sanitized_.front_brake_req_pct = sanitized_.brake * 100.0f;
    sanitized_.rear_brake_req_pct = sanitized_.front_brake_req_pct;
  }

  void update(const fsai::sim::svcu::dbc::Ai2VcuStatus& status) {
    status_ = status;
    has_status_ = true;
    recompute();
  }

  void update(const fsai::sim::svcu::dbc::Ai2VcuSteer& steer) {
    steer_deg_ = steer.steer_deg;
    has_steer_ = true;
    recompute();
  }

  void updateFront(const fsai::sim::svcu::dbc::Ai2VcuDrive& drive) {
    front_drive_ = drive;
    has_front_drive_ = true;
    recompute();
  }

  void updateRear(const fsai::sim::svcu::dbc::Ai2VcuDrive& drive) {
    rear_drive_ = drive;
    has_rear_drive_ = true;
    recompute();
  }

  void update(const fsai::sim::svcu::dbc::Ai2VcuBrake& brake) {
    brake_ = brake;
    has_brake_ = true;
    recompute();
  }

  const SanitizedCommand& sanitized() const { return sanitized_; }

  bool handshake() const { return has_status_ && status_.handshake; }
  bool estop_active() const { return has_status_ && status_.estop_request; }
  fsai::sim::svcu::dbc::DirectionRequest direction() const {
    return has_status_ ? status_.direction_request
                       : fsai::sim::svcu::dbc::DirectionRequest::kNeutral;
  }
  fsai::sim::svcu::dbc::MissionStatus mission_status() const {
    return has_status_ ? status_.mission_status
                       : fsai::sim::svcu::dbc::MissionStatus::kNotSelected;
  }
  float brake_front_bias() const { return brake_front_bias_; }
  float brake_rear_bias() const { return brake_rear_bias_; }
  float brake_max_force() const { return brake_max_force_; }
  const fsai::sim::svcu::dbc::Ai2VcuStatus& raw_status() const { return status_; }

 private:
  static float clamp01(float value) { return std::clamp(value, 0.0f, 1.0f); }

  void recompute() {
    SanitizedCommand out{};
    out.brake = max_brake_;
    out.front_brake_req_pct = out.brake * 100.0f;
    out.rear_brake_req_pct = out.front_brake_req_pct;
    out.throttle = 0.0f;
    out.steer_rad = 0.0f;
    out.enabled = false;

    const bool handshake = has_status_ && status_.handshake;
    const bool estop = has_status_ && status_.estop_request;
    const bool forward = !has_status_ ||
                         status_.direction_request ==
                             fsai::sim::svcu::dbc::DirectionRequest::kForward;

    if (handshake && forward && !estop) {
      float steer_deg = has_steer_ ? std::clamp(steer_deg_,
                                               -fsai::sim::svcu::dbc::kMaxSteerDeg,
                                               fsai::sim::svcu::dbc::kMaxSteerDeg)
                                   : 0.0f;
      float steer_rad = steer_deg * fsai::sim::svcu::dbc::kDegToRad;
      steer_rad = std::clamp(steer_rad, steer_min_, steer_max_);
      out.steer_rad = steer_rad;

      float front_req = has_front_drive_
                            ? std::clamp(front_drive_.axle_torque_request_nm, 0.0f,
                                         fsai::sim::svcu::dbc::kMaxAxleTorqueNm)
                            : 0.0f;
      float rear_req = has_rear_drive_
                           ? std::clamp(rear_drive_.axle_torque_request_nm, 0.0f,
                                        fsai::sim::svcu::dbc::kMaxAxleTorqueNm)
                           : 0.0f;
      float total_req = front_req + rear_req;
      float throttle = clamp01(total_req /
                               (2.0f * fsai::sim::svcu::dbc::kMaxAxleTorqueNm));
      if (throttle > max_throttle_ && total_req > 0.0f) {
        const float scale = max_throttle_ / std::max(throttle, 1e-6f);
        front_req *= scale;
        rear_req *= scale;
        throttle = max_throttle_;
      }
      out.throttle = throttle;
      out.front_torque_request_nm = front_req;
      out.rear_torque_request_nm = rear_req;

      float front_pct = has_brake_ ? std::clamp(brake_.front_pct, 0.0f,
                                                fsai::sim::svcu::dbc::kMaxBrakePercent)
                                   : 0.0f;
      float rear_pct = has_brake_ ? std::clamp(brake_.rear_pct, 0.0f,
                                               fsai::sim::svcu::dbc::kMaxBrakePercent)
                                  : 0.0f;
      float brake = clamp01(std::max(front_pct, rear_pct) / 100.0f);
      if (brake > max_brake_ && (front_pct > 0.0f || rear_pct > 0.0f)) {
        const float scale = (max_brake_ * 100.0f) /
                            std::max(1.0f, std::max(front_pct, rear_pct));
        front_pct *= scale;
        rear_pct *= scale;
        brake = max_brake_;
      }
      out.brake = brake;
      out.front_brake_req_pct = front_pct;
      out.rear_brake_req_pct = rear_pct;
      out.enabled = true;
    } else if (estop) {
      out.brake = max_brake_;
      out.front_brake_req_pct = fsai::sim::svcu::dbc::kMaxBrakePercent;
      out.rear_brake_req_pct = fsai::sim::svcu::dbc::kMaxBrakePercent;
    } else if (!handshake || !forward) {
      out.brake = max_brake_;
      out.front_brake_req_pct = out.brake * 100.0f;
      out.rear_brake_req_pct = out.front_brake_req_pct;
    }

    sanitized_ = out;
  }

  float accel_min_{0.0f};
  float accel_max_{0.0f};
  float steer_min_{0.0f};
  float steer_max_{0.0f};
  float brake_front_bias_{0.5f};
  float brake_rear_bias_{0.5f};
  float brake_max_force_{0.0f};
  float max_throttle_{1.0f};
  float max_brake_{1.0f};

  fsai::sim::svcu::dbc::Ai2VcuStatus status_{};
  fsai::sim::svcu::dbc::Ai2VcuDrive front_drive_{};
  fsai::sim::svcu::dbc::Ai2VcuDrive rear_drive_{};
  fsai::sim::svcu::dbc::Ai2VcuBrake brake_{};
  float steer_deg_{0.0f};

  bool has_status_{false};
  bool has_front_drive_{false};
  bool has_rear_drive_{false};
  bool has_brake_{false};
  bool has_steer_{false};

  SanitizedCommand sanitized_{};
};

}  // namespace

namespace fsai::sim::svcu {

RunStats RunSvcu(const RunConfig& config, const RunLimits& limits,
                 const RunCallbacks& callbacks) {
  RunStats stats{};

  VehicleParam params;
  try {
    params = VehicleParam::loadFromFile(config.vehicle_config);
  } catch (...) {
    return stats;
  }

  CommandAggregator aggregator(params);

  auto can_link = make_can_link(config.can_endpoint);
  if (!can_link || !can_link->open(config.can_endpoint, config.can_loopback)) {
    return stats;
  }

  fsai::sim::svcu::UdpEndpoint command_tx;
  if (!command_tx.connect(config.command_port)) {
    return stats;
  }

  fsai::sim::svcu::UdpEndpoint telemetry_rx;
  if (!telemetry_rx.bind(config.telemetry_port)) {
    return stats;
  }

  const auto should_continue = [&]() {
    if (limits.stop_flag && !limits.stop_flag->load(std::memory_order_relaxed)) {
      return false;
    }
    if (limits.deadline && std::chrono::steady_clock::now() >= *limits.deadline) {
      return false;
    }
    return true;
  };

  stats.ok = true;

  fsai::sim::svcu::CommandPacket cmd_packet{};
  fsai::sim::svcu::TelemetryPacket telemetry{};

  while (should_continue()) {
    while (auto frame_opt = can_link->receive()) {
      stats.can_frames_rx++;
      if (callbacks.on_can_rx) {
        callbacks.on_can_rx(*frame_opt);
      }
      const auto& frame = *frame_opt;
      switch (frame.can_id) {
        case fsai::sim::svcu::dbc::kMsgIdAi2VcuStatus: {
          fsai::sim::svcu::dbc::Ai2VcuStatus status{};
          if (fsai::sim::svcu::dbc::decode_ai2vcu_status(frame, status)) {
            aggregator.update(status);
          }
          break;
        }
        case fsai::sim::svcu::dbc::kMsgIdAi2VcuSteer: {
          fsai::sim::svcu::dbc::Ai2VcuSteer steer{};
          if (fsai::sim::svcu::dbc::decode_ai2vcu_steer(frame, steer)) {
            aggregator.update(steer);
          }
          break;
        }
        case fsai::sim::svcu::dbc::kMsgIdAi2VcuDriveFront: {
          fsai::sim::svcu::dbc::Ai2VcuDrive drive{};
          if (fsai::sim::svcu::dbc::decode_ai2vcu_drive_front(frame, drive)) {
            aggregator.updateFront(drive);
          }
          break;
        }
        case fsai::sim::svcu::dbc::kMsgIdAi2VcuDriveRear: {
          fsai::sim::svcu::dbc::Ai2VcuDrive drive{};
          if (fsai::sim::svcu::dbc::decode_ai2vcu_drive_rear(frame, drive)) {
            aggregator.updateRear(drive);
          }
          break;
        }
        case fsai::sim::svcu::dbc::kMsgIdAi2VcuBrake: {
          fsai::sim::svcu::dbc::Ai2VcuBrake brake{};
          if (fsai::sim::svcu::dbc::decode_ai2vcu_brake(frame, brake)) {
            aggregator.update(brake);
          }
          break;
        }
        default:
          break;
      }
    }

    const auto now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    const SanitizedCommand& sanitized = aggregator.sanitized();
    cmd_packet.t_ns = now_ns;
    cmd_packet.steer_rad = sanitized.steer_rad;
    cmd_packet.throttle = std::clamp(sanitized.throttle, 0.0f, 1.0f);
    cmd_packet.brake = std::clamp(sanitized.brake, 0.0f, 1.0f);
    cmd_packet.enabled = sanitized.enabled ? 1 : 0;
    if (command_tx.send(&cmd_packet, sizeof(cmd_packet))) {
      stats.command_packets_sent++;
      if (callbacks.on_command) {
        CommandSample sample{};
        sample.t_ns = cmd_packet.t_ns;
        sample.steer_rad = cmd_packet.steer_rad;
        sample.throttle = cmd_packet.throttle;
        sample.brake = cmd_packet.brake;
        sample.enabled = sanitized.enabled;
        sample.front_torque_request_nm = sanitized.front_torque_request_nm;
        sample.rear_torque_request_nm = sanitized.rear_torque_request_nm;
        sample.front_brake_req_pct = sanitized.front_brake_req_pct;
        sample.rear_brake_req_pct = sanitized.rear_brake_req_pct;
        sample.handshake = aggregator.handshake();
        sample.estop = aggregator.estop_active();
        callbacks.on_command(sample);
      }
    }

    while (auto bytes = telemetry_rx.receive(&telemetry, sizeof(telemetry))) {
      if (*bytes != sizeof(telemetry)) {
        continue;
      }
      stats.telemetry_packets_processed++;

      const SanitizedCommand& cmd = aggregator.sanitized();
      const bool handshake = aggregator.handshake();
      const bool estop = aggregator.estop_active();
      fsai::sim::svcu::dbc::Vcu2AiStatus status_msg{};
      status_msg.handshake = handshake;
      status_msg.shutdown_request = estop;
      status_msg.as_switch_on = handshake;
      status_msg.ts_switch_on = true;
      status_msg.go_signal = cmd.enabled;
      status_msg.as_state = estop
                                ? fsai::sim::svcu::dbc::AsState::kEmergencyBrake
                                : (cmd.enabled
                                       ? fsai::sim::svcu::dbc::AsState::kDriving
                                       : (handshake ? fsai::sim::svcu::dbc::AsState::kReady
                                                    : fsai::sim::svcu::dbc::AsState::kOff));
      status_msg.steering_status = cmd.enabled
                                       ? fsai::sim::svcu::dbc::SteeringStatus::kActive
                                       : fsai::sim::svcu::dbc::SteeringStatus::kOff;
      status_msg.fault = estop;
      status_msg.warning = false;
      status_msg.ami_state = 4;
      status_msg.ai_estop_request = estop;
      status_msg.hvil_open_fault = false;
      status_msg.hvil_short_fault = false;
      status_msg.ebs_fault = estop;
      status_msg.offboard_charger_fault = false;
      status_msg.ai_comms_lost = false;
      status_msg.warn_batt_temp_high = false;
      status_msg.warn_batt_soc_low = false;
      status_msg.charge_procedure_fault = false;
      status_msg.autonomous_braking_fault = false;
      status_msg.mission_status_fault = false;
      status_msg.shutdown_cause = estop ? 1 : 0;
      status_msg.bms_fault = false;
      status_msg.brake_plausibility_fault = false;
      if (callbacks.on_status) {
        callbacks.on_status(status_msg);
      }
      const auto status_frame = fsai::sim::svcu::dbc::encode_vcu2ai_status(status_msg);
      if (can_link->send(status_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(status_frame);
        }
      }

      fsai::sim::svcu::dbc::Vcu2AiSteer steer_msg{};
      steer_msg.angle_deg = telemetry.steer_angle_rad * fsai::sim::svcu::dbc::kRadToDeg;
      steer_msg.angle_max_deg = std::clamp(
          std::abs(static_cast<float>(params.input_ranges.delta.max) *
                   fsai::sim::svcu::dbc::kRadToDeg),
          0.0f, fsai::sim::svcu::dbc::kMaxSteerDeg);
      steer_msg.angle_request_deg = cmd.steer_rad * fsai::sim::svcu::dbc::kRadToDeg;
      const auto steer_frame = fsai::sim::svcu::dbc::encode_vcu2ai_steer(steer_msg);
      if (can_link->send(steer_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(steer_frame);
        }
      }

      fsai::sim::svcu::dbc::Vcu2AiDrive front_drive{};
      front_drive.axle_trq_nm = telemetry.front_axle_torque_nm;
      front_drive.axle_trq_request_nm = cmd.front_torque_request_nm;
      front_drive.axle_trq_max_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
      const auto front_drive_frame =
          fsai::sim::svcu::dbc::encode_vcu2ai_drive_front(front_drive);
      if (can_link->send(front_drive_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(front_drive_frame);
        }
      }

      fsai::sim::svcu::dbc::Vcu2AiDrive rear_drive{};
      rear_drive.axle_trq_nm = telemetry.rear_axle_torque_nm;
      rear_drive.axle_trq_request_nm = cmd.rear_torque_request_nm;
      rear_drive.axle_trq_max_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
      const auto rear_drive_frame =
          fsai::sim::svcu::dbc::encode_vcu2ai_drive_rear(rear_drive);
      if (can_link->send(rear_drive_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(rear_drive_frame);
        }
      }

      fsai::sim::svcu::dbc::Vcu2AiSpeeds speed_msg{};
      for (int i = 0; i < 4; ++i) {
        speed_msg.wheel_rpm[i] = telemetry.wheel_speed_rpm[i];
      }
      const auto speeds_frame =
          fsai::sim::svcu::dbc::encode_vcu2ai_speeds(speed_msg);
      if (can_link->send(speeds_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(speeds_frame);
        }
      }

      const double front_bias = std::max(0.0f, aggregator.brake_front_bias());
      const double rear_bias = std::max(0.0f, aggregator.brake_rear_bias());
      const double front_force_max = std::max(1.0, aggregator.brake_max_force() * front_bias);
      const double rear_force_max = std::max(1.0, aggregator.brake_max_force() * rear_bias);
      const double front_force_N = telemetry.brake_pressure_front_bar * 1000.0;
      const double rear_force_N = telemetry.brake_pressure_rear_bar * 1000.0;
      const float front_actual_pct = std::clamp(
          static_cast<float>((front_force_N / front_force_max) * 100.0), 0.0f,
          fsai::sim::svcu::dbc::kMaxBrakePercent);
      const float rear_actual_pct = std::clamp(
          static_cast<float>((rear_force_N / rear_force_max) * 100.0), 0.0f,
          fsai::sim::svcu::dbc::kMaxBrakePercent);

      fsai::sim::svcu::dbc::Vcu2AiBrake brake_msg{};
      brake_msg.front_pct = front_actual_pct;
      brake_msg.rear_pct = rear_actual_pct;
      brake_msg.front_req_pct = cmd.front_brake_req_pct;
      brake_msg.rear_req_pct = cmd.rear_brake_req_pct;
      brake_msg.status_brk = estop ? fsai::sim::svcu::dbc::BrakeStatus::kFault
                                   : (handshake
                                          ? fsai::sim::svcu::dbc::BrakeStatus::kReady
                                          : fsai::sim::svcu::dbc::BrakeStatus::kInitialising);
      brake_msg.status_ebs = estop
                                 ? fsai::sim::svcu::dbc::EbsStatus::kTriggered
                                 : (handshake
                                        ? fsai::sim::svcu::dbc::EbsStatus::kArmed
                                        : fsai::sim::svcu::dbc::EbsStatus::kUnavailable);
      const auto brake_frame =
          fsai::sim::svcu::dbc::encode_vcu2ai_brake(brake_msg);
      if (can_link->send(brake_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(brake_frame);
        }
      }

      fsai::sim::svcu::dbc::Vcu2LogDynamics1 dyn{};
      dyn.speed_actual_kph = telemetry.gps_speed_mps * 3.6f;
      dyn.speed_target_kph = aggregator.raw_status().veh_speed_demand_kph;
      dyn.steer_actual_deg = steer_msg.angle_deg;
      dyn.steer_target_deg = steer_msg.angle_request_deg;
      dyn.brake_actual_pct = std::clamp(std::max(front_actual_pct, rear_actual_pct),
                                        0.0f, fsai::sim::svcu::dbc::kMaxBrakePercent);
      dyn.brake_target_pct = std::clamp(
          std::max(cmd.front_brake_req_pct, cmd.rear_brake_req_pct), 0.0f,
          fsai::sim::svcu::dbc::kMaxBrakePercent);
      dyn.drive_trq_actual_pct = std::clamp(
          static_cast<float>((std::max(0.0f, telemetry.front_axle_torque_nm) +
                              std::max(0.0f, telemetry.rear_axle_torque_nm)) /
                             (2.0f * fsai::sim::svcu::dbc::kMaxAxleTorqueNm) *
                             100.0f),
          0.0f, 100.0f);
      dyn.drive_trq_target_pct = std::clamp(cmd.throttle * 100.0f, 0.0f, 100.0f);
      const auto dyn_frame =
          fsai::sim::svcu::dbc::encode_vcu2log_dynamics1(dyn);
      if (can_link->send(dyn_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(dyn_frame);
        }
      }

      fsai::sim::svcu::dbc::Ai2LogDynamics2 imu{};
      imu.accel_longitudinal_mps2 = telemetry.imu_ax_mps2;
      imu.accel_lateral_mps2 = telemetry.imu_ay_mps2;
      imu.yaw_rate_degps = telemetry.imu_yaw_rate_rps * fsai::sim::svcu::dbc::kRadToDeg;
      const auto imu_frame =
          fsai::sim::svcu::dbc::encode_ai2log_dynamics2(imu);
      if (can_link->send(imu_frame)) {
        stats.can_frames_tx++;
        if (callbacks.on_can_tx) {
          callbacks.on_can_tx(imu_frame);
        }
      }
    }

    if (limits.sleep_interval.count() > 0) {
      std::this_thread::sleep_for(limits.sleep_interval);
    }
  }

  return stats;
}

}  // namespace fsai::sim::svcu
