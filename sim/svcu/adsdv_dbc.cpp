#include "adsdv_dbc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace fsai::sim::svcu::dbc {
namespace {

template <typename T>
constexpr T clamp(T value, T min, T max) {
  return value < min ? min : (value > max ? max : value);
}

inline void write_u16_le(uint8_t* dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

inline uint16_t read_u16_le(const uint8_t* src) {
  return static_cast<uint16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

inline int16_t read_i16_le(const uint8_t* src) {
  return static_cast<int16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

inline uint32_t read_bits_le(const uint8_t* data, int start_bit, int length) {
  uint32_t value = 0;
  for (int i = 0; i < length; ++i) {
    const int bit_index = start_bit + i;
    const int byte_index = bit_index / 8;
    const int bit_in_byte = bit_index % 8;
    const uint8_t bit = (data[byte_index] >> bit_in_byte) & 0x1u;
    value |= static_cast<uint32_t>(bit) << i;
  }
  return value;
}

inline void write_bits_le(uint8_t* data, int start_bit, int length, uint32_t value) {
  for (int i = 0; i < length; ++i) {
    const int bit_index = start_bit + i;
    const int byte_index = bit_index / 8;
    const int bit_in_byte = bit_index % 8;
    const uint8_t mask = static_cast<uint8_t>(1u << bit_in_byte);
    if ((value >> i) & 0x1u) {
      data[byte_index] |= mask;
    } else {
      data[byte_index] &= static_cast<uint8_t>(~mask);
    }
  }
}

inline int16_t pack_i16(double value, double scale, double min, double max) {
  const double limited = clamp(value, min, max);
  const double raw = std::round(limited / scale);
  return static_cast<int16_t>(clamp(raw, -32768.0, 32767.0));
}

inline uint16_t pack_u16(double value, double scale, double min, double max) {
  const double limited = clamp(value, min, max);
  const double raw = std::round(limited / scale);
  return static_cast<uint16_t>(clamp(raw, 0.0, 65535.0));
}

inline uint8_t pack_u8(double value, double scale, double min, double max) {
  const double limited = clamp(value, min, max);
  const double raw = std::round(limited / scale);
  return static_cast<uint8_t>(clamp(raw, 0.0, 255.0));
}

inline int8_t pack_i8(double value, double scale, double min, double max) {
  const double limited = clamp(value, min, max);
  const double raw = std::round(limited / scale);
  return static_cast<int8_t>(clamp(raw, -128.0, 127.0));
}

}  // namespace

bool decode_ai2vcu_status(const can_frame& frame, Ai2VcuStatus& out) {
  if (frame.can_id != kMsgIdAi2VcuStatus || frame.can_dlc < 8) {
    return false;
  }

  std::memset(&out, 0, sizeof(out));
  out.handshake = read_bits_le(frame.data, 0, 1) != 0;
  out.estop_request = read_bits_le(frame.data, 8, 1) != 0;
  out.mission_status = static_cast<MissionStatus>(read_bits_le(frame.data, 12, 2));
  out.direction_request = static_cast<DirectionRequest>(read_bits_le(frame.data, 14, 2));
  out.lap_counter = static_cast<uint8_t>(read_bits_le(frame.data, 16, 4));
  out.cones_count_actual = static_cast<uint8_t>(read_bits_le(frame.data, 24, 8));
  out.cones_count_all = static_cast<uint16_t>(read_bits_le(frame.data, 32, 16));
  out.veh_speed_actual_kph = static_cast<float>(read_bits_le(frame.data, 48, 8));
  out.veh_speed_demand_kph = static_cast<float>(read_bits_le(frame.data, 56, 8));
  return true;
}

bool decode_ai2vcu_drive_front(const can_frame& frame, Ai2VcuDrive& out) {
  if (frame.can_id != kMsgIdAi2VcuDriveFront || frame.can_dlc < 4) {
    return false;
  }
  out.axle_torque_request_nm = static_cast<float>(read_u16_le(frame.data) * 0.1f);
  out.motor_speed_max_rpm = static_cast<float>(read_u16_le(frame.data + 2) * 1.0f);
  return true;
}

bool decode_ai2vcu_drive_rear(const can_frame& frame, Ai2VcuDrive& out) {
  if (frame.can_id != kMsgIdAi2VcuDriveRear || frame.can_dlc < 4) {
    return false;
  }
  out.axle_torque_request_nm = static_cast<float>(read_u16_le(frame.data) * 0.1f);
  out.motor_speed_max_rpm = static_cast<float>(read_u16_le(frame.data + 2) * 1.0f);
  return true;
}

bool decode_ai2vcu_steer(const can_frame& frame, Ai2VcuSteer& out) {
  if (frame.can_id != kMsgIdAi2VcuSteer || frame.can_dlc < 2) {
    return false;
  }
  out.steer_deg = static_cast<float>(read_i16_le(frame.data) * 0.1f);
  return true;
}

bool decode_ai2vcu_brake(const can_frame& frame, Ai2VcuBrake& out) {
  if (frame.can_id != kMsgIdAi2VcuBrake || frame.can_dlc < 2) {
    return false;
  }
  out.front_pct = static_cast<float>(frame.data[0]) * 0.5f;
  out.rear_pct = static_cast<float>(frame.data[1]) * 0.5f;
  return true;
}

can_frame encode_ai2vcu_status(const Ai2VcuStatus& status) {
  can_frame frame{};
  frame.can_id = kMsgIdAi2VcuStatus;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, sizeof(frame.data));

  write_bits_le(frame.data, 0, 1, status.handshake ? 1u : 0u);
  write_bits_le(frame.data, 8, 1, status.estop_request ? 1u : 0u);
  write_bits_le(frame.data, 12, 2, static_cast<uint32_t>(status.mission_status));
  write_bits_le(frame.data, 14, 2, static_cast<uint32_t>(status.direction_request));
  write_bits_le(frame.data, 16, 4, status.lap_counter);
  write_bits_le(frame.data, 24, 8, status.cones_count_actual);
  write_bits_le(frame.data, 32, 16, status.cones_count_all);
  write_bits_le(frame.data, 48, 8, pack_u8(status.veh_speed_actual_kph, 1.0, 0.0, 255.0));
  write_bits_le(frame.data, 56, 8, pack_u8(status.veh_speed_demand_kph, 1.0, 0.0, 255.0));
  return frame;
}

can_frame encode_ai2vcu_drive_front(const Ai2VcuDrive& drive) {
  can_frame frame{};
  frame.can_id = kMsgIdAi2VcuDriveFront;
  frame.can_dlc = 4;
  write_u16_le(frame.data, pack_u16(drive.axle_torque_request_nm, 0.1, 0.0, kMaxAxleTorqueNm));
  write_u16_le(frame.data + 2, pack_u16(drive.motor_speed_max_rpm, 1.0, 0.0, 4000.0));
  return frame;
}

can_frame encode_ai2vcu_drive_rear(const Ai2VcuDrive& drive) {
  can_frame frame{};
  frame.can_id = kMsgIdAi2VcuDriveRear;
  frame.can_dlc = 4;
  write_u16_le(frame.data, pack_u16(drive.axle_torque_request_nm, 0.1, 0.0, kMaxAxleTorqueNm));
  write_u16_le(frame.data + 2, pack_u16(drive.motor_speed_max_rpm, 1.0, 0.0, 4000.0));
  return frame;
}

can_frame encode_ai2vcu_steer(const Ai2VcuSteer& steer) {
  can_frame frame{};
  frame.can_id = kMsgIdAi2VcuSteer;
  frame.can_dlc = 2;
  const int16_t raw = pack_i16(steer.steer_deg, 0.1, -kMaxSteerDeg, kMaxSteerDeg);
  write_u16_le(frame.data, static_cast<uint16_t>(raw));
  return frame;
}

can_frame encode_ai2vcu_brake(const Ai2VcuBrake& brake) {
  can_frame frame{};
  frame.can_id = kMsgIdAi2VcuBrake;
  frame.can_dlc = 2;
  frame.data[0] = pack_u8(brake.front_pct, 0.5, 0.0, kMaxBrakePercent);
  frame.data[1] = pack_u8(brake.rear_pct, 0.5, 0.0, kMaxBrakePercent);
  return frame;
}

can_frame encode_vcu2ai_status(const Vcu2AiStatus& status) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiStatus;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, sizeof(frame.data));

  write_bits_le(frame.data, 0, 1, status.handshake ? 1u : 0u);
  write_bits_le(frame.data, 8, 1, status.shutdown_request ? 1u : 0u);
  write_bits_le(frame.data, 9, 1, status.as_switch_on ? 1u : 0u);
  write_bits_le(frame.data, 10, 1, status.ts_switch_on ? 1u : 0u);
  write_bits_le(frame.data, 11, 1, status.go_signal ? 1u : 0u);
  write_bits_le(frame.data, 12, 2, static_cast<uint32_t>(status.steering_status));
  write_bits_le(frame.data, 16, 4, static_cast<uint32_t>(status.as_state));
  write_bits_le(frame.data, 20, 4, clamp<uint32_t>(status.ami_state, 0u, 15u));
  write_bits_le(frame.data, 24, 1, status.fault ? 1u : 0u);
  write_bits_le(frame.data, 25, 1, status.warning ? 1u : 0u);
  write_bits_le(frame.data, 40, 1, status.ai_estop_request ? 1u : 0u);
  write_bits_le(frame.data, 41, 1, status.hvil_open_fault ? 1u : 0u);
  write_bits_le(frame.data, 42, 1, status.hvil_short_fault ? 1u : 0u);
  write_bits_le(frame.data, 43, 1, status.ebs_fault ? 1u : 0u);
  write_bits_le(frame.data, 44, 1, status.offboard_charger_fault ? 1u : 0u);
  write_bits_le(frame.data, 45, 1, status.ai_comms_lost ? 1u : 0u);
  write_bits_le(frame.data, 32, 1, status.warn_batt_temp_high ? 1u : 0u);
  write_bits_le(frame.data, 33, 1, status.warn_batt_soc_low ? 1u : 0u);
  write_bits_le(frame.data, 48, 1, status.charge_procedure_fault ? 1u : 0u);
  write_bits_le(frame.data, 46, 1, status.autonomous_braking_fault ? 1u : 0u);
  write_bits_le(frame.data, 47, 1, status.mission_status_fault ? 1u : 0u);
  write_bits_le(frame.data, 56, 8, status.shutdown_cause);
  write_bits_le(frame.data, 49, 1, status.bms_fault ? 1u : 0u);
  write_bits_le(frame.data, 50, 1, status.brake_plausibility_fault ? 1u : 0u);
  return frame;
}

can_frame encode_vcu2ai_steer(const Vcu2AiSteer& steer) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiSteer;
  frame.can_dlc = 6;
  const int16_t angle_raw = pack_i16(steer.angle_deg, 0.1, -kMaxSteerDeg, kMaxSteerDeg);
  const uint16_t max_raw = pack_u16(steer.angle_max_deg, 0.1, 0.0, kMaxSteerDeg);
  const int16_t request_raw = pack_i16(steer.angle_request_deg, 0.1, -kMaxSteerDeg, kMaxSteerDeg);
  write_u16_le(frame.data, static_cast<uint16_t>(angle_raw));
  write_u16_le(frame.data + 2, max_raw);
  write_u16_le(frame.data + 4, static_cast<uint16_t>(request_raw));
  return frame;
}

can_frame encode_vcu2ai_drive_front(const Vcu2AiDrive& drive) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiDriveFront;
  frame.can_dlc = 6;
  const int16_t trq_raw = pack_i16(drive.axle_trq_nm, 0.1, -kMaxAxleTorqueNm, kMaxAxleTorqueNm);
  const uint16_t trq_max_raw = pack_u16(drive.axle_trq_max_nm, 0.1, 0.0, kMaxAxleTorqueNm);
  const uint16_t trq_req_raw = pack_u16(drive.axle_trq_request_nm, 0.1, 0.0, kMaxAxleTorqueNm);
  write_u16_le(frame.data, static_cast<uint16_t>(trq_raw));
  write_u16_le(frame.data + 2, trq_req_raw);
  write_u16_le(frame.data + 4, trq_max_raw);
  return frame;
}

can_frame encode_vcu2ai_drive_rear(const Vcu2AiDrive& drive) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiDriveRear;
  frame.can_dlc = 6;
  const int16_t trq_raw = pack_i16(drive.axle_trq_nm, 0.1, -kMaxAxleTorqueNm, kMaxAxleTorqueNm);
  const uint16_t trq_req_raw = pack_u16(drive.axle_trq_request_nm, 0.1, 0.0, kMaxAxleTorqueNm);
  const uint16_t trq_max_raw = pack_u16(drive.axle_trq_max_nm, 0.1, 0.0, kMaxAxleTorqueNm);
  write_u16_le(frame.data, static_cast<uint16_t>(trq_raw));
  write_u16_le(frame.data + 2, trq_req_raw);
  write_u16_le(frame.data + 4, trq_max_raw);
  return frame;
}

can_frame encode_vcu2ai_speeds(const Vcu2AiSpeeds& speeds) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiSpeeds;
  frame.can_dlc = 8;
  for (int i = 0; i < 4; ++i) {
    write_u16_le(frame.data + (i * 2), pack_u16(speeds.wheel_rpm[i], 1.0, 0.0, 1250.0));
  }
  return frame;
}

can_frame encode_vcu2ai_brake(const Vcu2AiBrake& brake) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiBrake;
  frame.can_dlc = 5;
  frame.data[0] = pack_u8(brake.front_pct, 0.5, 0.0, kMaxBrakePercent);
  frame.data[1] = pack_u8(brake.front_req_pct, 0.5, 0.0, kMaxBrakePercent);
  frame.data[2] = pack_u8(brake.rear_pct, 0.5, 0.0, kMaxBrakePercent);
  frame.data[3] = pack_u8(brake.rear_req_pct, 0.5, 0.0, kMaxBrakePercent);
  frame.data[4] = static_cast<uint8_t>(
      (static_cast<uint8_t>(brake.status_brk) & 0xFu) |
      ((static_cast<uint8_t>(brake.status_ebs) & 0xFu) << 4));
  return frame;
}

can_frame encode_vcu2ai_wheel_counts(const Vcu2AiWheelCounts& counts) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2AiWheelCounts;
  frame.can_dlc = 8;
  for (int i = 0; i < 4; ++i) {
    write_u16_le(frame.data + (i * 2), counts.pulse_count[i]);
  }
  return frame;
}

can_frame encode_vcu2log_dynamics1(const Vcu2LogDynamics1& dyn) {
  can_frame frame{};
  frame.can_id = kMsgIdVcu2LogDynamics1;
  frame.can_dlc = 8;
  frame.data[0] = pack_u8(dyn.speed_actual_kph, 1.0, 0.0, 255.0);
  frame.data[1] = pack_u8(dyn.speed_target_kph, 1.0, 0.0, 255.0);
  frame.data[2] = static_cast<uint8_t>(pack_i8(dyn.steer_actual_deg, 0.5, -64.0, 63.5));
  frame.data[3] = static_cast<uint8_t>(pack_i8(dyn.steer_target_deg, 0.5, -64.0, 63.5));
  frame.data[4] = pack_u8(dyn.brake_actual_pct, 1.0, 0.0, 100.0);
  frame.data[5] = pack_u8(dyn.brake_target_pct, 1.0, 0.0, 100.0);
  frame.data[6] = pack_u8(dyn.drive_trq_actual_pct, 1.0, 0.0, 100.0);
  frame.data[7] = pack_u8(dyn.drive_trq_target_pct, 1.0, 0.0, 100.0);
  return frame;
}

can_frame encode_ai2log_dynamics2(const Ai2LogDynamics2& imu) {
  can_frame frame{};
  frame.can_id = kMsgIdAi2LogDynamics2;
  frame.can_dlc = 6;
  const int16_t ax_raw = pack_i16(imu.accel_longitudinal_mps2, 0.00195313, -64.0, 63.99804688);
  const int16_t ay_raw = pack_i16(imu.accel_lateral_mps2, 0.00195313, -64.0, 63.99804688);
  const int16_t yaw_raw = pack_i16(imu.yaw_rate_degps, 0.0078125, -256.0, 255.9921875);
  write_u16_le(frame.data, static_cast<uint16_t>(ax_raw));
  write_u16_le(frame.data + 2, static_cast<uint16_t>(ay_raw));
  write_u16_le(frame.data + 4, static_cast<uint16_t>(yaw_raw));
  return frame;
}

}  // namespace fsai::sim::svcu::dbc
