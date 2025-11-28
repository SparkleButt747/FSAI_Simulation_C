#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>

#include "link.hpp"
#include "can_link.hpp"
#include "svcu_runner.hpp"

namespace {
std::atomic_bool g_running{true};

void handle_signal(int) { g_running.store(false, std::memory_order_relaxed); }

uint16_t parse_u16(const char* value, uint16_t fallback) {
  if (!value) return fallback;
  char* end = nullptr;
  long v = std::strtol(value, &end, 10);
  if (end == value || v < 0 || v > 65535) return fallback;
  return static_cast<uint16_t>(v);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::string vehicle_config = "../configs/vehicle/ads-dv.yaml";
  std::string can_iface = fsai::sim::svcu::default_can_endpoint();
  uint16_t command_port = fsai::sim::svcu::kDefaultCommandPort;
  uint16_t telemetry_port = fsai::sim::svcu::kDefaultTelemetryPort;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--vehicle" && i + 1 < argc) {
      vehicle_config = argv[++i];
    } else if (arg == "--can-if" && i + 1 < argc) {
      can_iface = argv[++i];
    } else if (arg == "--cmd-port" && i + 1 < argc) {
      command_port = parse_u16(argv[++i], command_port);
    } else if (arg == "--state-port" && i + 1 < argc) {
      telemetry_port = parse_u16(argv[++i], telemetry_port);
    } else if (arg == "--help") {
      return 0;
    }
  }

  can_iface = fsai::sim::svcu::canonicalize_can_endpoint(can_iface);

  fsai::sim::svcu::RunConfig config{};
  config.vehicle_config = vehicle_config;
  config.can_endpoint = can_iface;
  config.command_port = command_port;
  config.telemetry_port = telemetry_port;
  config.can_loopback = false;

  fsai::sim::svcu::RunLimits limits{};
  limits.stop_flag = &g_running;
  limits.sleep_interval = std::chrono::milliseconds(1);

  const auto stats = fsai::sim::svcu::RunSvcu(config, limits);
  return stats.ok ? 0 : 1;
}
