#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cstring>

#if defined(__linux__)
#include <linux/rtnetlink.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "ai2vcu_adapter.hpp"
#include "can_iface.hpp"
#include "svcu_runner.hpp"
#include "VehicleParam.hpp"
#include "link.hpp"
#include "types.h"

#include <cmath>

namespace {
using SteadyClock = std::chrono::steady_clock;

bool DebugEnabled() {
  static const bool enabled = std::getenv("SVCU_TEST_DEBUG") != nullptr;
  return enabled;
}

void DebugLog(const char* message) {
  if (DebugEnabled() && message) {
    std::fprintf(stderr, "%s\n", message);
  }
}

bool Fail(const char* message) {
  DebugLog(message);
  return false;
}

#if defined(__linux__)
bool InterfaceExists(const std::string& iface) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return false;
  }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface.c_str());
  const bool ok = (::ioctl(fd, SIOCGIFFLAGS, &ifr) == 0);
  ::close(fd);
  return ok;
}

bool BringInterfaceUp(const std::string& iface) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return false;
  }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface.c_str());
  if (::ioctl(fd, SIOCGIFFLAGS, &ifr) != 0) {
    ::close(fd);
    return false;
  }
  ifr.ifr_flags |= IFF_UP;
  const bool ok = (::ioctl(fd, SIOCSIFFLAGS, &ifr) == 0);
  ::close(fd);
  return ok;
}

int AddAttribute(struct nlmsghdr* hdr, size_t max_len, int type, const void* data,
                 size_t data_len) {
  const size_t len = RTA_LENGTH(data_len);
  const size_t new_len = NLMSG_ALIGN(hdr->nlmsg_len) + RTA_ALIGN(len);
  if (new_len > max_len) {
    return -1;
  }
  auto* attr = reinterpret_cast<struct rtattr*>(reinterpret_cast<char*>(hdr) +
                                                NLMSG_ALIGN(hdr->nlmsg_len));
  attr->rta_type = type;
  attr->rta_len = len;
  if (data_len > 0 && data != nullptr) {
    std::memcpy(RTA_DATA(attr), data, data_len);
  }
  hdr->nlmsg_len = new_len;
  return 0;
}

struct rtattr* BeginNestedAttribute(struct nlmsghdr* hdr, size_t max_len, int type) {
  auto* nest = reinterpret_cast<struct rtattr*>(reinterpret_cast<char*>(hdr) +
                                                NLMSG_ALIGN(hdr->nlmsg_len));
  if (AddAttribute(hdr, max_len, type, nullptr, 0) != 0) {
    return nullptr;
  }
  return nest;
}

void EndNestedAttribute(struct nlmsghdr* hdr, struct rtattr* nest) {
  nest->rta_len = reinterpret_cast<char*>(hdr) + NLMSG_ALIGN(hdr->nlmsg_len) -
                  reinterpret_cast<char*>(nest);
}

bool CreateVcanInterface(const std::string& iface) {
  struct {
    struct nlmsghdr hdr;
    struct ifinfomsg ifinfo;
    char data[256];
  } req{};

  req.hdr.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
  req.hdr.nlmsg_type = RTM_NEWLINK;
  req.hdr.nlmsg_flags = NLM_F_REQUEST | NLM_F_CREATE | NLM_F_EXCL | NLM_F_ACK;
  req.hdr.nlmsg_seq = 1;
  req.ifinfo.ifi_family = AF_UNSPEC;
  req.ifinfo.ifi_change = 0xFFFFFFFF;

  if (AddAttribute(&req.hdr, sizeof(req), IFLA_IFNAME, iface.c_str(),
                   iface.size() + 1) != 0) {
    return false;
  }

  auto* linkinfo = BeginNestedAttribute(&req.hdr, sizeof(req), IFLA_LINKINFO);
  if (!linkinfo) {
    return false;
  }
  static constexpr char kVcanKind[] = "vcan";
  if (AddAttribute(&req.hdr, sizeof(req), IFLA_INFO_KIND, kVcanKind,
                   sizeof(kVcanKind)) != 0) {
    return false;
  }
  EndNestedAttribute(&req.hdr, linkinfo);

  int fd = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (fd < 0) {
    return false;
  }

  sockaddr_nl addr{};
  addr.nl_family = AF_NETLINK;
  if (::sendto(fd, &req, req.hdr.nlmsg_len, 0,
               reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return false;
  }

  bool success = false;
  std::array<char, 512> buffer{};
  const ssize_t len = ::recv(fd, buffer.data(), buffer.size(), 0);
  if (len >= static_cast<ssize_t>(sizeof(struct nlmsghdr))) {
    const auto* resp = reinterpret_cast<const struct nlmsghdr*>(buffer.data());
    if (resp->nlmsg_type == NLMSG_ERROR) {
      const auto* err = reinterpret_cast<const struct nlmsgerr*>(NLMSG_DATA(resp));
      success = (err->error == 0);
    } else {
      success = true;
    }
  }
  ::close(fd);
  return success;
}

bool TryLoadVcanModule() {
  static const char* const kCandidates[] = {
      "/sbin/modprobe", "/usr/sbin/modprobe", "/bin/modprobe", "/usr/bin/modprobe"};
  for (const char* path : kCandidates) {
    if (std::filesystem::exists(path)) {
      const std::string cmd = std::string(path) + " vcan > /dev/null 2>&1";
      const int rc = std::system(cmd.c_str());
      if (rc == 0) {
        return true;
      }
    }
  }
  return false;
}

bool EnsureVcan(const std::string& iface) {
  if (InterfaceExists(iface)) {
    return BringInterfaceUp(iface);
  }
  TryLoadVcanModule();
  if (!CreateVcanInterface(iface)) {
    return false;
  }
  return BringInterfaceUp(iface);
}
#else
bool EnsureVcan(const std::string& iface) {
  (void)iface;
  return false;
}
#endif

uint64_t NowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    SteadyClock::now().time_since_epoch())
                                    .count());
}

struct BaselineEntry {
  float steer_rad{0.0f};
  float throttle{0.0f};
  float brake{0.0f};
  bool enabled{false};
  bool handshake{false};
  bool estop{false};
  float front_brake_req_pct{0.0f};
  float rear_brake_req_pct{0.0f};
  float front_torque_request_nm{0.0f};
  float rear_torque_request_nm{0.0f};
};

std::vector<BaselineEntry> LoadBaseline(const std::filesystem::path& path) {
  std::ifstream file(path);
  std::vector<BaselineEntry> entries;
  if (!file.is_open()) {
    return entries;
  }
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    BaselineEntry entry{};
    int enabled = 0;
    int handshake = 0;
    int estop = 0;
    char comma = 0;
    std::stringstream ss(line);
    ss >> entry.steer_rad >> comma >> entry.throttle >> comma >> entry.brake >> comma >> enabled >> comma >> handshake >> comma >> estop >> comma >> entry.front_brake_req_pct >> comma >> entry.rear_brake_req_pct >> comma >> entry.front_torque_request_nm >> comma >> entry.rear_torque_request_nm;
    if (ss.fail()) {
      entries.clear();
      return entries;
    }
    entry.enabled = enabled != 0;
    entry.handshake = handshake != 0;
    entry.estop = estop != 0;
    entries.push_back(entry);
  }
  return entries;
}

bool ApproximatelyEqual(float a, float b, float tol) {
  return std::abs(a - b) <= tol;
}

fsai::sim::svcu::dbc::Vcu2AiStatus MakeStatus(fsai::sim::svcu::dbc::AsState as_state,
                                              bool go_signal = true) {
  fsai::sim::svcu::dbc::Vcu2AiStatus status{};
  status.as_state = as_state;
  status.go_signal = go_signal;
  return status;
}

fsai::control::runtime::Ai2VcuAdapter::AdapterTelemetry MakeTelemetry(float speed_mps,
                                                                      uint8_t lap_counter) {
  fsai::control::runtime::Ai2VcuAdapter::AdapterTelemetry telemetry{};
  telemetry.measured_speed_mps = speed_mps;
  telemetry.lap_counter = lap_counter;
  telemetry.mission_selected = true;
  telemetry.mission_running = true;
  telemetry.mission_finished = false;
  return telemetry;
}

struct TimedStatus {
  fsai::sim::svcu::dbc::Vcu2AiStatus status;
  SteadyClock::time_point time;
};

VehicleParam LoadVehicleParam(const std::filesystem::path& path) {
  return VehicleParam::loadFromFile(path.string());
}

fsai::control::runtime::Ai2VcuAdapterConfig BuildAdapterConfig(const VehicleParam& params) {
  fsai::control::runtime::Ai2VcuAdapterConfig cfg{};
  cfg.front_torque_fraction = static_cast<float>(params.powertrain.torque_split_front);
  cfg.rear_torque_fraction = static_cast<float>(params.powertrain.torque_split_rear);
  cfg.front_axle_max_torque_nm = static_cast<float>(params.powertrain.torque_front_max_nm);
  cfg.rear_axle_max_torque_nm = static_cast<float>(params.powertrain.torque_rear_max_nm);

  float front_bias = static_cast<float>(params.brakes.front_bias);
  float rear_bias = static_cast<float>(params.brakes.rear_bias);
  const float sum = front_bias + rear_bias;
  if (sum > 0.0f) {
    front_bias = std::clamp(front_bias / sum, 0.0f, 1.0f);
  } else {
    front_bias = 0.5f;
  }
  cfg.brake_front_bias = front_bias;
  cfg.brake_rear_bias = std::clamp(1.0f - front_bias, 0.0f, 1.0f);
  cfg.max_speed_kph = static_cast<float>(params.input_ranges.vel.max * 3.6);
  return cfg;
}

bool TestMissionStatusPackaging() {
  const std::filesystem::path repo_root = std::filesystem::path(FSAI_PROJECT_ROOT);
  const auto vehicle_config = repo_root / "configs" / "vehicle" / "configDry.yaml";

  VehicleParam params;
  try {
    params = LoadVehicleParam(vehicle_config);
  } catch (...) {
    return Fail("mission status packaging vehicle load failed");
  }

  auto cfg = BuildAdapterConfig(params);
  cfg.mission_descriptor = fsai::sim::MissionDescriptor{};
  cfg.mission_descriptor->type = fsai::sim::MissionType::kTrackdrive;
  cfg.mission_descriptor->name = "Trackdrive";
  cfg.mission_descriptor->short_name = "track";

  fsai::control::runtime::Ai2VcuAdapter adapter(cfg);

  fsai::types::ControlCmd cmd{};
  cmd.t_ns = NowNs();
  auto ready_status = MakeStatus(fsai::sim::svcu::dbc::AsState::kReady, false);

  fsai::control::runtime::Ai2VcuAdapter::AdapterTelemetry telemetry{};
  telemetry.mission_selected = true;
  telemetry.mission_running = false;
  telemetry.mission_finished = false;
  telemetry.mission_laps_target = 3;
  telemetry.mission_laps_completed = 0;

  auto selected = adapter.Adapt(cmd, ready_status, telemetry);
  if (selected.status.mission_status != fsai::sim::svcu::dbc::MissionStatus::kSelected) {
    return Fail("mission status did not enter selected state");
  }
  if (selected.status.mission_id == 0) {
    return Fail("mission id missing in selected state");
  }
  if (selected.status.mission_complete) {
    return Fail("mission complete flagged during selection");
  }

  telemetry.mission_running = true;
  auto running = adapter.Adapt(cmd, MakeStatus(fsai::sim::svcu::dbc::AsState::kDriving), telemetry);
  if (running.status.mission_status != fsai::sim::svcu::dbc::MissionStatus::kRunning) {
    return Fail("mission status did not enter running state");
  }
  if (running.status.direction_request != fsai::sim::svcu::dbc::DirectionRequest::kForward) {
    return Fail("direction request not forward while running");
  }

  telemetry.mission_running = false;
  telemetry.mission_finished = true;
  telemetry.mission_laps_completed = 3;
  auto finished = adapter.Adapt(cmd, MakeStatus(fsai::sim::svcu::dbc::AsState::kDriving), telemetry);
  if (finished.status.mission_status != fsai::sim::svcu::dbc::MissionStatus::kFinished) {
    return Fail("mission status did not enter finished state");
  }
  if (!finished.status.mission_complete) {
    return Fail("mission complete flag missing");
  }
  if (finished.status.direction_request != fsai::sim::svcu::dbc::DirectionRequest::kNeutral) {
    return Fail("direction request not neutral after finish");
  }

  return true;
}

bool TestMessageRatesAndSaturation() {
  const bool has_vcan = EnsureVcan("vcan0");
  const std::string can_endpoint = has_vcan ? std::string("vcan0")
                                             : std::string("udp:47001@47002");
  const std::string control_endpoint = has_vcan ? std::string("vcan0")
                                                 : std::string("udp:47002@47001");
  DebugLog("TestMessageRatesAndSaturation start");
  if (!has_vcan) {
    DebugLog("Using UDP fallback for message rate test");
  }

  const auto test_dir = std::filesystem::path(__FILE__).parent_path();
  const std::filesystem::path repo_root = std::filesystem::path(FSAI_PROJECT_ROOT);
  const auto vehicle_config = repo_root / "configs" / "vehicle" / "configDry.yaml";
  const auto baseline_path = test_dir / "baseline" / "control_loop_trace.csv";

  const VehicleParam params = LoadVehicleParam(vehicle_config);
  const auto baseline = LoadBaseline(baseline_path);
  if (baseline.empty()) {
    return Fail("baseline load failed");
  }

  fsai::control::runtime::Ai2VcuAdapter adapter(BuildAdapterConfig(params));

  std::atomic_bool running{true};
  fsai::sim::svcu::RunConfig run_config{};
  run_config.vehicle_config = vehicle_config.string();
  run_config.can_endpoint = can_endpoint;
  run_config.command_port = 51001;
  run_config.telemetry_port = 51002;
  run_config.can_loopback = false;

  fsai::sim::svcu::RunLimits limits{};
  limits.stop_flag = &running;
  limits.sleep_interval = std::chrono::milliseconds(1);
  limits.deadline = SteadyClock::now() + std::chrono::seconds(5);

  std::mutex sample_mutex;
  std::condition_variable sample_cv;
  std::vector<fsai::sim::svcu::CommandSample> command_samples;
  std::vector<TimedStatus> status_samples;
  std::vector<SteadyClock::time_point> status_times;

  fsai::sim::svcu::RunCallbacks callbacks{};
  callbacks.on_command = [&](const fsai::sim::svcu::CommandSample& sample) {
    std::lock_guard<std::mutex> lock(sample_mutex);
    command_samples.push_back(sample);
    if (DebugEnabled()) {
      char buffer[192];
      std::snprintf(buffer, sizeof(buffer),
                    "cmd enabled=%d throttle=%.3f brake=%.3f steer=%.3f handshake=%d estop=%d",
                    sample.enabled ? 1 : 0, sample.throttle, sample.brake, sample.steer_rad,
                    sample.handshake ? 1 : 0, sample.estop ? 1 : 0);
      DebugLog(buffer);
    }
    sample_cv.notify_all();
  };
  callbacks.on_status = [&](const fsai::sim::svcu::dbc::Vcu2AiStatus& status) {
    std::lock_guard<std::mutex> lock(sample_mutex);
    status_samples.push_back({status, SteadyClock::now()});
    if (DebugEnabled()) {
      char buffer[192];
      std::snprintf(buffer, sizeof(buffer),
                    "status go=%d estop=%d state=%d handshake=%d",
                    status.go_signal ? 1 : 0, status.shutdown_request ? 1 : 0,
                    static_cast<int>(status.as_state), status.handshake ? 1 : 0);
      DebugLog(buffer);
    }
    status_times.push_back(status_samples.back().time);
    sample_cv.notify_all();
  };

  fsai::sim::svcu::RunStats stats{};
  std::thread svcu_thread([&]() { stats = fsai::sim::svcu::RunSvcu(run_config, limits, callbacks); });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  fsai::sim::svcu::UdpEndpoint telemetry_tx;
  if (!telemetry_tx.connect(run_config.telemetry_port)) {
    running.store(false);
    if (svcu_thread.joinable()) svcu_thread.join();
    return Fail("telemetry connect failed");
  }

  fsai::control::runtime::CanIface::Config can_cfg{};
  can_cfg.mode = fsai::control::runtime::CanIface::Mode::kSimulation;
  can_cfg.endpoint = control_endpoint;
  can_cfg.enable_loopback = false;
  can_cfg.wheel_radius_m = params.tire.radius;

  fsai::control::runtime::CanIface can_iface;
  if (!can_iface.Initialize(can_cfg)) {
    running.store(false);
    if (svcu_thread.joinable()) svcu_thread.join();
    return Fail("can iface init failed");
  }

  const int iterations = 40;
  const auto period = std::chrono::milliseconds(10);
  auto next_tick = SteadyClock::now();

  for (int i = 0; i < iterations; ++i) {
    next_tick += period;

    fsai::types::ControlCmd cmd{};
    cmd.t_ns = NowNs();
    cmd.steer_rad = 0.5f;
    cmd.throttle = 1.2f;
    cmd.brake = 1.3f;

    auto commands = adapter.Adapt(
        cmd, MakeStatus(fsai::sim::svcu::dbc::AsState::kDriving),
        MakeTelemetry(0.0f, 1));
    commands.steer.steer_deg = 60.0f;
    commands.front_drive.axle_torque_request_nm =
        fsai::sim::svcu::dbc::kMaxAxleTorqueNm * 1.5f;
    commands.rear_drive.axle_torque_request_nm =
        fsai::sim::svcu::dbc::kMaxAxleTorqueNm * 1.5f;
    commands.brake.front_pct = fsai::sim::svcu::dbc::kMaxBrakePercent * 1.5f;
    commands.brake.rear_pct = commands.brake.front_pct;
    if (DebugEnabled()) {
      char buffer[192];
      std::snprintf(buffer, sizeof(buffer),
                    "adapted handshake=%d direction=%d throttle=%.3f brake=%.3f",
                    commands.status.handshake ? 1 : 0,
                    static_cast<int>(commands.status.direction_request), cmd.throttle,
                    cmd.brake);
      DebugLog(buffer);
    }
    can_iface.Send(commands, NowNs());

    fsai::sim::svcu::TelemetryPacket telemetry{};
    telemetry.t_ns = NowNs();
    telemetry_tx.send(&telemetry, sizeof(telemetry));

    can_iface.Poll(NowNs());
    std::this_thread::sleep_until(next_tick);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  telemetry_tx.close();
  can_iface.Shutdown();
  running.store(false);
  if (svcu_thread.joinable()) {
    svcu_thread.join();
  }

  if (!stats.ok) {
    return Fail("svcu stats not ok");
  }

  std::vector<fsai::sim::svcu::CommandSample> samples_copy;
  std::vector<SteadyClock::time_point> status_times_copy;
  {
    std::lock_guard<std::mutex> lock(sample_mutex);
    samples_copy = command_samples;
    status_times_copy = status_times;
  }

  std::vector<fsai::sim::svcu::CommandSample> enabled_samples;
  for (const auto& sample : samples_copy) {
    if (sample.enabled) {
      enabled_samples.push_back(sample);
    }
  }

  if (enabled_samples.size() < baseline.size()) {
    if (DebugEnabled()) {
      char buffer[192];
      std::snprintf(buffer, sizeof(buffer),
                    "enabled_samples=%zu baseline=%zu total=%zu",
                    enabled_samples.size(), baseline.size(), samples_copy.size());
      DebugLog(buffer);
    }
    return Fail("insufficient enabled samples");
  }

  for (size_t i = 0; i < baseline.size(); ++i) {
    const auto& actual = enabled_samples[i];
    const auto& expected = baseline[i];
    if (!ApproximatelyEqual(actual.steer_rad, expected.steer_rad, 1e-3f) ||
        !ApproximatelyEqual(actual.throttle, expected.throttle, 1e-3f) ||
        !ApproximatelyEqual(actual.brake, expected.brake, 1e-3f) ||
        !ApproximatelyEqual(actual.front_brake_req_pct, expected.front_brake_req_pct, 1e-2f) ||
        !ApproximatelyEqual(actual.rear_brake_req_pct, expected.rear_brake_req_pct, 1e-2f) ||
        !ApproximatelyEqual(actual.front_torque_request_nm, expected.front_torque_request_nm, 1e-2f) ||
        !ApproximatelyEqual(actual.rear_torque_request_nm, expected.rear_torque_request_nm, 1e-2f) ||
        actual.enabled != expected.enabled || actual.handshake != expected.handshake ||
        actual.estop != expected.estop) {
      return Fail("baseline mismatch");
    }
  }

  if (status_times_copy.size() < 6) {
    return Fail("insufficient status samples");
  }

  std::vector<double> intervals_ms;
  for (size_t i = 1; i < status_times_copy.size(); ++i) {
    const auto delta = std::chrono::duration<double, std::milli>(status_times_copy[i] - status_times_copy[i - 1]).count();
    intervals_ms.push_back(delta);
  }
  if (intervals_ms.empty()) {
    return Fail("no status intervals");
  }
  double sum = 0.0;
  for (double value : intervals_ms) {
    if (value < 5.0 || value > 20.0) {
      return Fail("status interval out of bounds");
    }
    sum += value;
  }
  const double avg = sum / static_cast<double>(intervals_ms.size());
  if (avg < 8.0 || avg > 12.0) {
    return Fail("status average interval out of bounds");
  }

  return true;
}

bool TestWatchdogTransitions() {
  const bool has_vcan = EnsureVcan("vcan0");
  const std::string can_endpoint = has_vcan ? std::string("vcan0")
                                             : std::string("udp:47003@47004");
  const std::string control_endpoint = has_vcan ? std::string("vcan0")
                                                 : std::string("udp:47004@47003");
  DebugLog("TestWatchdogTransitions start");
  if (!has_vcan) {
    DebugLog("Using UDP fallback for watchdog test");
  }

  const auto test_dir = std::filesystem::path(__FILE__).parent_path();
  const std::filesystem::path repo_root = std::filesystem::path(FSAI_PROJECT_ROOT);
  const auto vehicle_config = repo_root / "configs" / "vehicle" / "configDry.yaml";
  const VehicleParam params = LoadVehicleParam(vehicle_config);

  fsai::control::runtime::Ai2VcuAdapter adapter(BuildAdapterConfig(params));

  std::atomic_bool running{true};
  fsai::sim::svcu::RunConfig run_config{};
  run_config.vehicle_config = vehicle_config.string();
  run_config.can_endpoint = can_endpoint;
  run_config.command_port = 51003;
  run_config.telemetry_port = 51004;
  run_config.can_loopback = false;

  fsai::sim::svcu::RunLimits limits{};
  limits.stop_flag = &running;
  limits.sleep_interval = std::chrono::milliseconds(1);
  limits.deadline = SteadyClock::now() + std::chrono::seconds(5);

  std::mutex sample_mutex;
  std::condition_variable sample_cv;
  std::vector<fsai::sim::svcu::CommandSample> command_samples;
  std::vector<fsai::sim::svcu::dbc::Vcu2AiStatus> status_samples;

  fsai::sim::svcu::RunCallbacks callbacks{};
  callbacks.on_command = [&](const fsai::sim::svcu::CommandSample& sample) {
    std::lock_guard<std::mutex> lock(sample_mutex);
    command_samples.push_back(sample);
    sample_cv.notify_all();
  };
  callbacks.on_status = [&](const fsai::sim::svcu::dbc::Vcu2AiStatus& status) {
    std::lock_guard<std::mutex> lock(sample_mutex);
    status_samples.push_back(status);
    sample_cv.notify_all();
  };

  fsai::sim::svcu::RunStats stats{};
  std::thread svcu_thread([&]() { stats = fsai::sim::svcu::RunSvcu(run_config, limits, callbacks); });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  fsai::sim::svcu::UdpEndpoint telemetry_tx;
  if (!telemetry_tx.connect(run_config.telemetry_port)) {
    running.store(false);
    if (svcu_thread.joinable()) svcu_thread.join();
    return Fail("watchdog telemetry connect failed");
  }

  fsai::control::runtime::CanIface::Config can_cfg{};
  can_cfg.mode = fsai::control::runtime::CanIface::Mode::kSimulation;
  can_cfg.endpoint = control_endpoint;
  can_cfg.enable_loopback = false;
  can_cfg.wheel_radius_m = params.tire.radius;

  fsai::control::runtime::CanIface can_iface;
  if (!can_iface.Initialize(can_cfg)) {
    running.store(false);
    if (svcu_thread.joinable()) svcu_thread.join();
    return Fail("watchdog can iface init failed");
  }

  auto run_stage = [&](const fsai::control::runtime::Ai2VcuCommandSet& commands, int iterations) {
    const auto period = std::chrono::milliseconds(10);
    auto next_tick = SteadyClock::now();
    for (int i = 0; i < iterations; ++i) {
      next_tick += period;
      can_iface.Send(commands, NowNs());
      fsai::sim::svcu::TelemetryPacket telemetry{};
      telemetry.t_ns = NowNs();
      telemetry_tx.send(&telemetry, sizeof(telemetry));
      can_iface.Poll(NowNs());
      std::this_thread::sleep_until(next_tick);
    }
  };

  fsai::types::ControlCmd cmd{};
  cmd.steer_rad = 0.5f;
  cmd.throttle = 1.0f;
  cmd.brake = 1.0f;
  cmd.t_ns = NowNs();
  auto driving_commands = adapter.Adapt(
      cmd, MakeStatus(fsai::sim::svcu::dbc::AsState::kDriving),
      MakeTelemetry(0.0f, 1));
  driving_commands.steer.steer_deg = 20.0f;
  driving_commands.front_drive.axle_torque_request_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
  driving_commands.rear_drive.axle_torque_request_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
  driving_commands.brake.front_pct = fsai::sim::svcu::dbc::kMaxBrakePercent;
  driving_commands.brake.rear_pct = fsai::sim::svcu::dbc::kMaxBrakePercent;

  run_stage(driving_commands, 20);

  {
    std::lock_guard<std::mutex> lock(sample_mutex);
    if (command_samples.empty() || status_samples.empty()) {
      telemetry_tx.close();
      can_iface.Shutdown();
      running.store(false);
      if (svcu_thread.joinable()) svcu_thread.join();
      return Fail("watchdog driving stage missing samples");
    }
    const auto last_sample = command_samples.back();
    const auto last_status = status_samples.back();
    if (!last_sample.enabled || last_status.as_state != fsai::sim::svcu::dbc::AsState::kDriving ||
        !last_status.go_signal) {
      telemetry_tx.close();
      can_iface.Shutdown();
      running.store(false);
      if (svcu_thread.joinable()) svcu_thread.join();
      return Fail("watchdog driving state mismatch");
    }
  }

  fsai::types::ControlCmd hold_cmd{};
  hold_cmd.t_ns = NowNs();
  auto handshake_drop = adapter.Adapt(
      hold_cmd, MakeStatus(fsai::sim::svcu::dbc::AsState::kReady, false),
      MakeTelemetry(0.0f, 1));
  handshake_drop.status.handshake = false;
  handshake_drop.status.direction_request = fsai::sim::svcu::dbc::DirectionRequest::kNeutral;
  handshake_drop.front_drive.axle_torque_request_nm = 0.0f;
  handshake_drop.rear_drive.axle_torque_request_nm = 0.0f;
  handshake_drop.brake.front_pct = 0.0f;
  handshake_drop.brake.rear_pct = 0.0f;

  run_stage(handshake_drop, 10);

  {
    std::lock_guard<std::mutex> lock(sample_mutex);
    if (command_samples.empty() || status_samples.empty()) {
      telemetry_tx.close();
      can_iface.Shutdown();
      running.store(false);
      if (svcu_thread.joinable()) svcu_thread.join();
      return Fail("watchdog handshake drop missing samples");
    }
    const auto last_sample = command_samples.back();
    const auto last_status = status_samples.back();
    if (last_sample.enabled || last_sample.throttle > 0.0f ||
        last_status.as_state != fsai::sim::svcu::dbc::AsState::kOff || last_status.go_signal) {
      telemetry_tx.close();
      can_iface.Shutdown();
      running.store(false);
      if (svcu_thread.joinable()) svcu_thread.join();
      return Fail("watchdog handshake drop state mismatch");
    }
  }

  fsai::types::ControlCmd estop_cmd{};
  estop_cmd.t_ns = NowNs();
  auto estop_commands = adapter.Adapt(
      estop_cmd, MakeStatus(fsai::sim::svcu::dbc::AsState::kEmergencyBrake),
      MakeTelemetry(0.0f, 1));
  estop_commands.status.estop_request = true;
  estop_commands.status.handshake = true;
  estop_commands.front_drive.axle_torque_request_nm = 0.0f;
  estop_commands.rear_drive.axle_torque_request_nm = 0.0f;
  estop_commands.brake.front_pct = 0.0f;
  estop_commands.brake.rear_pct = 0.0f;

  run_stage(estop_commands, 10);

  {
    std::lock_guard<std::mutex> lock(sample_mutex);
    if (command_samples.empty() || status_samples.empty()) {
      telemetry_tx.close();
      can_iface.Shutdown();
      running.store(false);
      if (svcu_thread.joinable()) svcu_thread.join();
      return Fail("watchdog estop missing samples");
    }
    const auto last_sample = command_samples.back();
    const auto last_status = status_samples.back();
    if (last_sample.enabled || !ApproximatelyEqual(last_sample.brake, 1.0f, 1e-3f) ||
        last_status.as_state != fsai::sim::svcu::dbc::AsState::kEmergencyBrake ||
        !last_status.shutdown_request || !last_status.ai_estop_request || last_status.go_signal) {
      telemetry_tx.close();
      can_iface.Shutdown();
      running.store(false);
      if (svcu_thread.joinable()) svcu_thread.join();
      return Fail("watchdog estop state mismatch");
    }
  }

  telemetry_tx.close();
  can_iface.Shutdown();
  running.store(false);
  if (svcu_thread.joinable()) {
    svcu_thread.join();
  }

  return stats.ok ? true : Fail("watchdog stats not ok");
}

}  // namespace

int main() {
  if (!TestMissionStatusPackaging()) {
    return 1;
  }
  if (!TestMessageRatesAndSaturation()) {
    return 1;
  }
  if (!TestWatchdogTransitions()) {
    return 1;
  }
  return 0;
}
