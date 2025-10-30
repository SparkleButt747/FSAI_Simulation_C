#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "adsdv_dbc.hpp"
#include "can_defs.hpp"

namespace fsai::sim::svcu {

struct RunConfig {
  std::string vehicle_config;
  std::string can_endpoint;
  uint16_t command_port{0};
  uint16_t telemetry_port{0};
  bool can_loopback{false};
};

struct RunLimits {
  std::atomic_bool* stop_flag{nullptr};
  std::optional<std::chrono::steady_clock::time_point> deadline{};
  std::chrono::milliseconds sleep_interval{std::chrono::milliseconds{1}};
};

struct CommandSample {
  uint64_t t_ns{0};
  float steer_rad{0.0f};
  float throttle{0.0f};
  float brake{0.0f};
  bool enabled{false};
  float front_torque_request_nm{0.0f};
  float rear_torque_request_nm{0.0f};
  float front_brake_req_pct{0.0f};
  float rear_brake_req_pct{0.0f};
  bool handshake{false};
  bool estop{false};
};

struct RunCallbacks {
  std::function<void(const CommandSample&)> on_command;
  std::function<void(const can_frame&)> on_can_tx;
  std::function<void(const can_frame&)> on_can_rx;
  std::function<void(const fsai::sim::svcu::dbc::Vcu2AiStatus&)> on_status;
};

struct RunStats {
  bool ok{false};
  uint64_t command_packets_sent{0};
  uint64_t telemetry_packets_processed{0};
  uint64_t can_frames_tx{0};
  uint64_t can_frames_rx{0};
};

RunStats RunSvcu(const RunConfig& config, const RunLimits& limits,
                 const RunCallbacks& callbacks = {});

}  // namespace fsai::sim::svcu

