#pragma once

#include "can_defs.hpp"

#include <cstdint>

namespace fsai::sim::svcu::dbc {

// Message identifiers (decimal from ADS-DV DBC)
constexpr uint32_t kMsgIdVcu2AiStatus = 1312;
constexpr uint32_t kMsgIdVcu2AiSteer = 1315;
constexpr uint32_t kMsgIdVcu2AiDriveFront = 1313;
constexpr uint32_t kMsgIdVcu2AiDriveRear = 1314;
constexpr uint32_t kMsgIdVcu2AiSpeeds = 1317;
constexpr uint32_t kMsgIdVcu2AiBrake = 1316;
constexpr uint32_t kMsgIdVcu2AiWheelCounts = 1318;
constexpr uint32_t kMsgIdVcu2LogDynamics1 = 1280;
constexpr uint32_t kMsgIdAi2LogDynamics2 = 1281;

constexpr uint32_t kMsgIdAi2VcuStatus = 1296;
constexpr uint32_t kMsgIdAi2VcuDriveFront = 1297;
constexpr uint32_t kMsgIdAi2VcuDriveRear = 1298;
constexpr uint32_t kMsgIdAi2VcuSteer = 1299;
constexpr uint32_t kMsgIdAi2VcuBrake = 1300;

// Physical limits encoded in the DBC
constexpr float kMaxSteerDeg = 21.0f;
constexpr float kMaxAxleTorqueNm = 195.0f;
constexpr float kMaxBrakePercent = 100.0f;
constexpr float kDegToRad = 0.0174532925199432957692369077f;
constexpr float kRadToDeg = 57.2957795130823208767981548f;

enum class DirectionRequest : uint8_t { kNeutral = 0, kForward = 1, kReverse = 2 };
enum class MissionStatus : uint8_t { kNotSelected = 0, kSelected = 1, kRunning = 2, kFinished = 3 };
enum class AsState : uint8_t {
  kOff = 1,
  kReady = 2,
  kDriving = 3,
  kEmergencyBrake = 4,
  kFinished = 5,
  kR2d = 6
};
enum class SteeringStatus : uint8_t { kOff = 0, kActive = 1, kFault = 2, kUnknown = 3 };
enum class BrakeStatus : uint8_t {
  kInitialising = 0,
  kReady = 1,
  kShuttingDown = 2,
  kShutdownComplete = 3,
  kFault = 4
};
enum class EbsStatus : uint8_t { kUnavailable = 1, kArmed = 2, kTriggered = 3 };

struct Ai2VcuStatus {
  bool handshake{false};
  bool estop_request{false};
  MissionStatus mission_status{MissionStatus::kNotSelected};
  DirectionRequest direction_request{DirectionRequest::kNeutral};
  uint8_t lap_counter{0};
  uint8_t cones_count_actual{0};
  uint16_t cones_count_all{0};
  float veh_speed_actual_kph{0.0f};
  float veh_speed_demand_kph{0.0f};
};

struct Ai2VcuDrive {
  float axle_torque_request_nm{0.0f};
  float motor_speed_max_rpm{0.0f};
};

struct Ai2VcuSteer {
  float steer_deg{0.0f};
};

struct Ai2VcuBrake {
  float front_pct{0.0f};
  float rear_pct{0.0f};
};

struct Vcu2AiStatus {
  bool handshake{false};
  bool shutdown_request{false};
  bool as_switch_on{false};
  bool ts_switch_on{false};
  bool go_signal{false};
  AsState as_state{AsState::kOff};
  SteeringStatus steering_status{SteeringStatus::kOff};
  bool fault{false};
  bool warning{false};
  uint8_t ami_state{0};
  bool ai_estop_request{false};
  bool hvil_open_fault{false};
  bool hvil_short_fault{false};
  bool ebs_fault{false};
  bool offboard_charger_fault{false};
  bool ai_comms_lost{false};
  bool warn_batt_temp_high{false};
  bool warn_batt_soc_low{false};
  bool charge_procedure_fault{false};
  bool autonomous_braking_fault{false};
  bool mission_status_fault{false};
  uint8_t shutdown_cause{0};
  bool bms_fault{false};
  bool brake_plausibility_fault{false};
};

struct Vcu2AiSteer {
  float angle_deg{0.0f};
  float angle_max_deg{kMaxSteerDeg};
  float angle_request_deg{0.0f};
};

struct Vcu2AiDrive {
  float axle_trq_nm{0.0f};
  float axle_trq_max_nm{kMaxAxleTorqueNm};
  float axle_trq_request_nm{0.0f};
};

struct Vcu2AiBrake {
  float front_pct{0.0f};
  float front_req_pct{0.0f};
  float rear_pct{0.0f};
  float rear_req_pct{0.0f};
  BrakeStatus status_brk{BrakeStatus::kInitialising};
  EbsStatus status_ebs{EbsStatus::kUnavailable};
};

struct Vcu2AiSpeeds {
  float wheel_rpm[4]{};  // FL, FR, RL, RR
};

struct Vcu2AiWheelCounts {
  uint16_t pulse_count[4]{};  // FL, FR, RL, RR
};

struct Vcu2LogDynamics1 {
  float speed_actual_kph{0.0f};
  float speed_target_kph{0.0f};
  float steer_actual_deg{0.0f};
  float steer_target_deg{0.0f};
  float brake_actual_pct{0.0f};
  float brake_target_pct{0.0f};
  float drive_trq_actual_pct{0.0f};
  float drive_trq_target_pct{0.0f};
};

struct Ai2LogDynamics2 {
  float accel_longitudinal_mps2{0.0f};
  float accel_lateral_mps2{0.0f};
  float yaw_rate_degps{0.0f};
};

// Decode helpers for AI -> VCU traffic
bool decode_ai2vcu_status(const can_frame& frame, Ai2VcuStatus& out);
bool decode_ai2vcu_drive_front(const can_frame& frame, Ai2VcuDrive& out);
bool decode_ai2vcu_drive_rear(const can_frame& frame, Ai2VcuDrive& out);
bool decode_ai2vcu_steer(const can_frame& frame, Ai2VcuSteer& out);
bool decode_ai2vcu_brake(const can_frame& frame, Ai2VcuBrake& out);

// Encode helpers for AI -> VCU traffic (used by simulator AI loop)
can_frame encode_ai2vcu_status(const Ai2VcuStatus& status);
can_frame encode_ai2vcu_drive_front(const Ai2VcuDrive& drive);
can_frame encode_ai2vcu_drive_rear(const Ai2VcuDrive& drive);
can_frame encode_ai2vcu_steer(const Ai2VcuSteer& steer);
can_frame encode_ai2vcu_brake(const Ai2VcuBrake& brake);

// Encode helpers for VCU -> AI traffic (produced by S-VCU)
can_frame encode_vcu2ai_status(const Vcu2AiStatus& status);
can_frame encode_vcu2ai_steer(const Vcu2AiSteer& steer);
can_frame encode_vcu2ai_drive_front(const Vcu2AiDrive& drive);
can_frame encode_vcu2ai_drive_rear(const Vcu2AiDrive& drive);
can_frame encode_vcu2ai_speeds(const Vcu2AiSpeeds& speeds);
can_frame encode_vcu2ai_brake(const Vcu2AiBrake& brake);
can_frame encode_vcu2ai_wheel_counts(const Vcu2AiWheelCounts& counts);
can_frame encode_vcu2log_dynamics1(const Vcu2LogDynamics1& dyn);
can_frame encode_ai2log_dynamics2(const Ai2LogDynamics2& imu);

}  // namespace fsai::sim::svcu::dbc
