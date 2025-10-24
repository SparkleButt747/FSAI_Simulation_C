#include <algorithm>
#include <array>
#include <chrono>
#include <numbers>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <SDL.h>
#include <yaml-cpp/yaml.h>

#include "Graphics.h"
#include "budget.h"
#include "fsai_clock.h"
#include "csv_logger.hpp"
#include "provider_registry.hpp"
#include "stereo_display.hpp"
#include "terminal_keyboard.hpp"
#include "sim_stereo_source.hpp"
#include "types.h"
#include "World.hpp"
#include "adsdv_dbc.hpp"
#include "link.hpp"
#include "can_link.hpp"
#include "ai2vcu_adapter.hpp"
#include "can_iface.hpp"

namespace {
constexpr double kDefaultDt = 0.01;
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr int kReportIntervalFrames = 120;
constexpr float kRenderScale = 5.0f;
constexpr uint16_t kDefaultCommandPort = fsai::sim::svcu::kDefaultCommandPort;
constexpr uint16_t kDefaultTelemetryPort = fsai::sim::svcu::kDefaultTelemetryPort;
constexpr double kCommandStaleSeconds = 0.1;
constexpr double kBaseLatitudeDeg = 37.4275;
constexpr double kBaseLongitudeDeg = -122.1697;
constexpr double kMetersPerDegreeLat = 111111.0;
constexpr const char* kDefaultSensorConfig = "../configs/sim/sensors.yaml";

inline double metersToLatitude(double north_meters) {
  return kBaseLatitudeDeg + north_meters / kMetersPerDegreeLat;
}

inline double metersToLongitude(double east_meters) {
  const double cos_lat = std::cos(kBaseLatitudeDeg * std::numbers::pi / 180.0);
  const double meters_per_degree_lon =
      kMetersPerDegreeLat * std::max(0.1, cos_lat);
  return kBaseLongitudeDeg + east_meters / meters_per_degree_lon;
}

struct ChannelNoiseConfig {
  double noise_std{0.0};
  double latency_s{0.0};
};

struct TorqueNoiseConfig {
  double front_noise_std_nm{0.0};
  double rear_noise_std_nm{0.0};
  double latency_s{0.0};
};

struct ImuNoiseConfig {
  double accel_longitudinal_std{0.0};
  double accel_lateral_std{0.0};
  double yaw_rate_std_degps{0.0};
  double latency_s{0.0};
};

struct GpsNoiseConfig {
  double lat_std_deg{0.0};
  double lon_std_deg{0.0};
  double speed_std_mps{0.0};
  double latency_s{0.0};
};

struct SensorNoiseConfig {
  uint32_t seed{1337u};
  ChannelNoiseConfig steering_deg{};
  ChannelNoiseConfig wheel_rpm{};
  TorqueNoiseConfig drive_torque{};
  ChannelNoiseConfig brake_pct{};
  ChannelNoiseConfig speed_kph{};
  ImuNoiseConfig imu{};
  GpsNoiseConfig gps{};
};

template <typename T>
class SensorDelayLine {
 public:
  void set_latency(uint64_t latency_ns) { latency_ns_ = latency_ns; }

  void push(uint64_t timestamp_ns, const T& value) {
    queue_.emplace_back(timestamp_ns, value);
  }

  std::optional<T> poll(uint64_t now_ns) {
    if (latency_ns_ == 0) {
      if (!queue_.empty()) {
        last_value_ = queue_.back().second;
        queue_.clear();
      }
      return last_value_;
    }
    while (!queue_.empty()) {
      const auto& front = queue_.front();
      if (front.first + latency_ns_ <= now_ns) {
        last_value_ = front.second;
        queue_.pop_front();
      } else {
        break;
      }
    }
    return last_value_;
  }

 private:
  uint64_t latency_ns_{0};
  std::deque<std::pair<uint64_t, T>> queue_{};
  std::optional<T> last_value_{};
};

struct TorqueSample {
  float front_nm{0.0f};
  float rear_nm{0.0f};
};

struct BrakeSample {
  float front_pct{0.0f};
  float rear_pct{0.0f};
  float front_req_pct{0.0f};
  float rear_req_pct{0.0f};
};

struct GpsSample {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double speed_mps{0.0};
};

SensorNoiseConfig LoadSensorNoiseConfig(const std::string& path) {
  SensorNoiseConfig cfg;
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (auto seed = root["seed"]) {
      cfg.seed = seed.as<uint32_t>(cfg.seed);
    }
    if (auto steering = root["steering"]) {
      cfg.steering_deg.noise_std = steering["noise_std_deg"].as<double>(cfg.steering_deg.noise_std);
      if (auto latency = steering["latency_ms"]) {
        cfg.steering_deg.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto wheel = root["wheel_speed"]) {
      cfg.wheel_rpm.noise_std = wheel["noise_std_rpm"].as<double>(cfg.wheel_rpm.noise_std);
      if (auto latency = wheel["latency_ms"]) {
        cfg.wheel_rpm.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto torque = root["drive_torque"]) {
      cfg.drive_torque.front_noise_std_nm = torque["front_noise_std_nm"].as<double>(cfg.drive_torque.front_noise_std_nm);
      cfg.drive_torque.rear_noise_std_nm = torque["rear_noise_std_nm"].as<double>(cfg.drive_torque.rear_noise_std_nm);
      if (auto latency = torque["latency_ms"]) {
        cfg.drive_torque.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto brake = root["brake"]) {
      cfg.brake_pct.noise_std = brake["noise_std_pct"].as<double>(cfg.brake_pct.noise_std);
      if (auto latency = brake["latency_ms"]) {
        cfg.brake_pct.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto speed = root["speed"]) {
      cfg.speed_kph.noise_std = speed["noise_std_kph"].as<double>(cfg.speed_kph.noise_std);
      if (auto latency = speed["latency_ms"]) {
        cfg.speed_kph.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto imu = root["imu"]) {
      cfg.imu.accel_longitudinal_std = imu["noise_std_ax"].as<double>(cfg.imu.accel_longitudinal_std);
      cfg.imu.accel_lateral_std = imu["noise_std_ay"].as<double>(cfg.imu.accel_lateral_std);
      cfg.imu.yaw_rate_std_degps = imu["noise_std_yaw_degps"].as<double>(cfg.imu.yaw_rate_std_degps);
      if (auto latency = imu["latency_ms"]) {
        cfg.imu.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto gps = root["gps"]) {
      cfg.gps.lat_std_deg = gps["noise_std_lat_deg"].as<double>(cfg.gps.lat_std_deg);
      cfg.gps.lon_std_deg = gps["noise_std_lon_deg"].as<double>(cfg.gps.lon_std_deg);
      cfg.gps.speed_std_mps = gps["noise_std_speed_mps"].as<double>(cfg.gps.speed_std_mps);
      if (auto latency = gps["latency_ms"]) {
        cfg.gps.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Failed to load sensor config '%s': %s\n", path.c_str(), e.what());
  }
  return cfg;
}
}  // namespace

int main(int argc, char* argv[]) {
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  auto parse_port = [](const char* text, uint16_t fallback) {
    if (!text) return fallback;
    char* end = nullptr;
    long value = std::strtol(text, &end, 10);
    if (end == text || value < 0 || value > 65535) {
      return fallback;
    }
    return static_cast<uint16_t>(value);
  };

  double dt = kDefaultDt;
  std::string can_iface = fsai::sim::svcu::default_can_endpoint();
  std::string mode = "sim";
  bool can_iface_overridden = false;
  uint16_t command_port = kDefaultCommandPort;
  uint16_t telemetry_port = kDefaultTelemetryPort;
  std::string sensor_config_path = kDefaultSensorConfig;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--dt" && i + 1 < argc) {
      dt = std::atof(argv[++i]);
    } else if (arg == "--can-if" && i + 1 < argc) {
      can_iface = argv[++i];
      can_iface_overridden = true;
    } else if (arg == "--mode" && i + 1 < argc) {
      mode = argv[++i];
    } else if (arg == "--cmd-port" && i + 1 < argc) {
      command_port = parse_port(argv[++i], command_port);
    } else if (arg == "--state-port" && i + 1 < argc) {
      telemetry_port = parse_port(argv[++i], telemetry_port);
    } else if (arg == "--sensor-config" && i + 1 < argc) {
      sensor_config_path = argv[++i];
    } else if (arg == "--help") {
      std::printf(
          "Usage: fsai_run [--dt seconds] [--can-if iface|udp:port] [--cmd-port p] "
          "[--state-port p] [--sensor-config path]\n");
      return EXIT_SUCCESS;
    } else if (i == 1 && argc == 2) {
      dt = std::atof(arg.c_str());
    } else {
      std::printf("Ignoring unrecognized argument '%s'\n", arg.c_str());
    }
  }

  if (dt <= 0.0) {
    std::printf("Invalid dt value provided. Using default dt = %.3f\n",
                kDefaultDt);
    dt = kDefaultDt;
  }
  if (!can_iface_overridden) {
    if (mode == "sim") {
      can_iface = "vcan0";
    } else if (mode == "car") {
      can_iface = "can0";
    }
  }
  can_iface = fsai::sim::svcu::canonicalize_can_endpoint(can_iface);
  const bool can_is_udp = fsai::sim::svcu::is_udp_endpoint(can_iface);
  std::printf(
      "Using dt = %.4f seconds, mode=%s, CAN %s (%s), cmd-port %u, state-port %u\n",
      dt, mode.c_str(), can_iface.c_str(), can_is_udp ? "udp" : "socketcan",
      command_port, telemetry_port);

  SensorNoiseConfig sensor_cfg = LoadSensorNoiseConfig(sensor_config_path);
  std::mt19937 noise_rng(sensor_cfg.seed);
  auto sample_noise = [&noise_rng](double stddev) {
    if (stddev <= 0.0) {
      return 0.0;
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(noise_rng);
  };

  fsai_clock_config clock_cfg{};
  clock_cfg.mode = FSAI_CLOCK_MODE_SIMULATED;
  clock_cfg.start_time_ns = 0;
  fsai_clock_init(clock_cfg);

  const uint64_t step_ns = fsai_clock_from_seconds(dt);
  const double step_seconds = fsai_clock_to_seconds(step_ns);
  const uint64_t ai2vcu_period_ns = fsai_clock_from_seconds(0.01);
  uint64_t last_ai2vcu_tx_ns = 0;

  SensorDelayLine<float> steer_delay;
  SensorDelayLine<std::array<float, 4>> wheel_delay;
  SensorDelayLine<TorqueSample> torque_delay;
  SensorDelayLine<BrakeSample> brake_delay;
  SensorDelayLine<float> speed_delay;
  SensorDelayLine<fsai::sim::svcu::dbc::Ai2LogDynamics2> imu_delay;
  SensorDelayLine<GpsSample> gps_delay;

  steer_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.steering_deg.latency_s));
  wheel_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.wheel_rpm.latency_s));
  torque_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.drive_torque.latency_s));
  brake_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.brake_pct.latency_s));
  speed_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.speed_kph.latency_s));
  imu_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.imu.latency_s));
  gps_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.gps.latency_s));

  float steer_meas_deg = 0.0f;
  bool has_steer_meas = false;
  std::array<float, 4> wheel_meas_rpm{0.0f, 0.0f, 0.0f, 0.0f};
  bool has_wheel_meas = false;
  TorqueSample torque_meas{};
  bool has_torque_meas = false;
  BrakeSample brake_meas{};
  bool has_brake_meas = false;
  float speed_meas_kph = 0.0f;
  bool has_speed_meas = false;
  fsai::sim::svcu::dbc::Ai2LogDynamics2 imu_meas{};
  bool has_imu_meas = false;
  GpsSample gps_meas{};
  bool has_gps_meas = false;

  fsai_budget_init();
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_SIMULATION, "Simulation Renderer",
                        fsai_clock_from_seconds(1.0 / 60.0));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CONTROL, "Planner + Controller",
                        fsai_clock_from_seconds(0.005));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_VISION, "Vision Pipeline",
                        fsai_clock_from_seconds(0.020));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CAN, "CAN Dispatch",
                        fsai_clock_from_seconds(0.002));
  fsai_budget_mark_unimplemented(
      FSAI_BUDGET_SUBSYSTEM_VISION,
      "Vision pipeline not yet integrated; timing scaffold active.");
  fsai_budget_mark_unimplemented(
      FSAI_BUDGET_SUBSYSTEM_CAN,
      "CAN transport not wired; pending hardware integration.");

  World world;
  world.init("../configs/vehicle/configDry.yaml");

  Graphics graphics{};
  if (Graphics_Init(&graphics, "Car Simulation 2D", kWindowWidth,
                    kWindowHeight) != 0) {
    std::fprintf(stderr, "Graphics_Init failed\n");
    return EXIT_FAILURE;
  }

  fsai::sim::integration::TerminalKeyboard keyboard{};
  fsai::sim::integration::CsvLogger logger("CarStateLog.csv", "RALog.csv");
  if (!logger.valid()) {
    std::fprintf(stderr, "Failed to open CSV logs\n");
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }

  fsai::sim::integration::registerBuiltInStereoProviders();
  auto stereo_factory =
      fsai::sim::integration::lookupStereoProvider("sim_stereo");
  std::unique_ptr<fsai::io::camera::sim_stereo::SimStereoSource> stereo_source;
  if (stereo_factory) {
    stereo_source = stereo_factory();
  }
  fsai::sim::integration::StereoDisplay stereo_display;

  std::vector<std::array<float, 3>> cone_positions;
  bool running = true;
  size_t frame_counter = 0;

  const VehicleParam& vehicle_param = world.model().param();

  fsai::control::runtime::CanIface::Config can_cfg{};
  can_cfg.endpoint = can_iface;
  can_cfg.enable_loopback = true;
  can_cfg.mode = (mode == "car") ? fsai::control::runtime::CanIface::Mode::kFsAiApi
                                   : fsai::control::runtime::CanIface::Mode::kSimulation;
  if (can_cfg.mode == fsai::control::runtime::CanIface::Mode::kFsAiApi) {
    can_cfg.enable_loopback = false;
  }
  can_cfg.wheel_radius_m = vehicle_param.tire.radius;

  fsai::control::runtime::CanIface can_interface;
  if (!can_interface.Initialize(can_cfg)) {
    if (mode == "sim" && !can_iface_overridden) {
      const std::string fallback = fsai::sim::svcu::default_can_endpoint();
      std::printf("Falling back to CAN endpoint %s\n", fallback.c_str());
      can_cfg.endpoint = fallback;
      can_cfg.mode = fsai::control::runtime::CanIface::Mode::kSimulation;
      if (!can_interface.Initialize(can_cfg)) {
        std::fprintf(stderr, "Failed to open CAN endpoint %s\n", can_cfg.endpoint.c_str());
        return EXIT_FAILURE;
      }
    } else {
      std::fprintf(stderr, "Failed to open CAN endpoint %s\n", can_cfg.endpoint.c_str());
      return EXIT_FAILURE;
    }
  }

  fsai::sim::svcu::UdpEndpoint command_rx;
  if (!command_rx.bind(command_port)) {
    std::fprintf(stderr, "Failed to bind command UDP port %u\n", command_port);
    return EXIT_FAILURE;
  }

  fsai::sim::svcu::UdpEndpoint telemetry_tx;
  if (!telemetry_tx.connect(telemetry_port)) {
    std::fprintf(stderr, "Failed to connect telemetry UDP port %u\n",
                 telemetry_port);
    return EXIT_FAILURE;
  }

  const int front_motor_count = std::max(0, vehicle_param.powertrain.front_motor_count);
  const int rear_motor_count = std::max(0, vehicle_param.powertrain.rear_motor_count);
  const int total_motors = std::max(1, front_motor_count + rear_motor_count);
  float tmp_front_motor_weight =
      total_motors > 0 ? static_cast<float>(front_motor_count) / total_motors
                       : 0.5f;
  tmp_front_motor_weight = std::clamp(tmp_front_motor_weight, 0.0f, 1.0f);
  if (tmp_front_motor_weight <= 0.0f &&
      std::clamp(1.0f - tmp_front_motor_weight, 0.0f, 1.0f) <= 0.0f) {
    tmp_front_motor_weight = 0.5f;
  }
  const float front_motor_weight = tmp_front_motor_weight;
  const float rear_motor_weight = std::clamp(1.0f - front_motor_weight, 0.0f, 1.0f);

  float tmp_brake_front_bias = static_cast<float>(vehicle_param.brakes.front_bias);
  float tmp_brake_rear_bias = static_cast<float>(vehicle_param.brakes.rear_bias);
  const float brake_sum = tmp_brake_front_bias + tmp_brake_rear_bias;
  if (brake_sum > 0.0f) {
    tmp_brake_front_bias = std::clamp(tmp_brake_front_bias / brake_sum, 0.0f, 1.0f);
  } else {
    tmp_brake_front_bias = 0.5f;
  }
  const float brake_front_bias = tmp_brake_front_bias;
  const float brake_rear_bias = std::clamp(1.0f - brake_front_bias, 0.0f, 1.0f);

  fsai::control::runtime::Ai2VcuAdapterConfig adapter_cfg{};
  adapter_cfg.front_motor_weight = front_motor_weight;
  adapter_cfg.rear_motor_weight = rear_motor_weight;
  adapter_cfg.brake_front_bias = brake_front_bias;
  adapter_cfg.brake_rear_bias = brake_rear_bias;
  adapter_cfg.max_speed_kph =
      static_cast<float>(vehicle_param.input_ranges.vel.max * 3.6);
  fsai::control::runtime::Ai2VcuAdapter ai2vcu_adapter(adapter_cfg);

  fsai::control::runtime::Ai2VcuCommandSet last_ai2vcu_commands{};
  bool has_last_ai_command = false;

  std::optional<fsai::sim::svcu::CommandPacket> latest_command;
  const uint64_t stale_threshold_ns =
      fsai_clock_from_seconds(kCommandStaleSeconds);


  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }
    if (!running) {
      break;
    }

    const int key = keyboard.poll();
    if (key == 'q' || key == 'Q') {
      running = false;
    } else if (key == 'm' || key == 'M') {
      world.useRacingAlgorithm = world.useRacingAlgorithm ? 0 : 1;
      std::printf("Switched to %s control mode.\n",
                  world.useRacingAlgorithm ? "automatic" : "manual");
    } else if (!world.useRacingAlgorithm && key != -1) {
      if (key == 'w' || key == 'W') {
        world.throttleInput = 1.0f;
        world.brakeInput = 0.0f;
      } else if (key == 's' || key == 'S') {
        world.throttleInput = 0.0f;
        world.brakeInput = 1.0f;
      }
      if (key == 'a' || key == 'A') {
        world.steeringAngle = -1.0f;
      } else if (key == 'd' || key == 'D') {
        world.steeringAngle = 1.0f;
      }
    }

    can_interface.Poll(fsai_clock_now());

    float autopThrottle = world.throttleInput;
    float autopBrake = world.brakeInput;
    float autopSteer = world.steeringAngle;
    if (world.useRacingAlgorithm) {
      float raThrottle = 0.0f;
      float raSteer = 0.0f;
      if (world.computeRacingControl(step_seconds, raThrottle, raSteer)) {
        autopSteer = raSteer;
        if (raThrottle >= 0.0f) {
          autopThrottle = raThrottle;
          autopBrake = 0.0f;
        } else {
          autopThrottle = 0.0f;
          autopBrake = -raThrottle;
        }
      }
    }

    const VehicleState& veh_state = world.vehicleState();
    const double vx = veh_state.velocity.x();
    const double vy = veh_state.velocity.y();
    const float speed_mps = static_cast<float>(std::sqrt(vx * vx + vy * vy));
    float measured_speed_mps = speed_mps;
    if (auto can_state = can_interface.LatestVehicleState()) {
      measured_speed_mps = std::max(can_state->vx, measured_speed_mps);
    }

    fsai::sim::svcu::CommandPacket command_packet{};
    while (auto bytes = command_rx.receive(&command_packet, sizeof(command_packet))) {
      if (*bytes == sizeof(command_packet)) {
        latest_command = command_packet;
      }
    }

    const uint64_t now_ns = fsai_clock_advance(step_ns);

    float appliedThrottle = autopThrottle;
    float appliedBrake = autopBrake;
    float appliedSteer = autopSteer;
    if (latest_command) {
      if (latest_command->t_ns <= now_ns &&
          now_ns - latest_command->t_ns <= stale_threshold_ns) {
        appliedThrottle = latest_command->throttle;
        appliedBrake = latest_command->brake;
        appliedSteer = latest_command->steer_rad;
      }
    }

    appliedThrottle = std::clamp(appliedThrottle, 0.0f, 1.0f);
    appliedBrake = std::clamp(appliedBrake, 0.0f, 1.0f);

    fsai::types::ControlCmd control_cmd{};
    control_cmd.throttle = appliedThrottle;
    control_cmd.brake = appliedBrake;
    control_cmd.steer_rad = appliedSteer;
    control_cmd.t_ns = now_ns;

    if (now_ns - last_ai2vcu_tx_ns >= ai2vcu_period_ns) {
      auto command_frames = ai2vcu_adapter.Adapt(
          control_cmd, measured_speed_mps, world.useRacingAlgorithm,
          static_cast<uint8_t>(std::clamp(world.completedLaps(), 0, 15)));
      if (can_interface.Send(command_frames)) {
        last_ai2vcu_tx_ns = now_ns;
        last_ai2vcu_commands = command_frames;
        has_last_ai_command = true;
      }
    }

    can_interface.Poll(now_ns);

    world.setSvcuCommand(appliedThrottle, appliedBrake, appliedSteer);
    world.throttleInput = appliedThrottle;
    world.brakeInput = appliedBrake;
    world.steeringAngle = appliedSteer;

    {
      fsai::time::ControlStageTimer control_timer("world_update");
      world.update(step_seconds);
    }
    const double sim_time_s = fsai_clock_to_seconds(now_ns);

    const auto& vehicle_state = world.vehicleState();
    const auto& model = world.model();
    const auto& pt_status = model.lastPowertrainStatus();
    const auto& brake_status = model.lastBrakeStatus();
    const WheelsInfo& wheel_info = world.wheelsInfo();
    const double wheel_radius = std::max(0.01, model.param().tire.radius);
    const double front_force = pt_status.front_drive_force - pt_status.front_regen_force -
                               brake_status.front_force;
    const double rear_force = pt_status.rear_drive_force - pt_status.rear_regen_force -
                              brake_status.rear_force;

    const double vehicle_speed_mps = std::sqrt(
        vehicle_state.velocity.x() * vehicle_state.velocity.x() +
        vehicle_state.velocity.y() * vehicle_state.velocity.y());
    const float actual_speed_kph = static_cast<float>(vehicle_speed_mps * 3.6);
    const float actual_steer_deg = static_cast<float>(wheel_info.steering *
        fsai::sim::svcu::dbc::kRadToDeg);

    steer_delay.push(now_ns, static_cast<float>(actual_steer_deg +
                                                sample_noise(sensor_cfg.steering_deg.noise_std)));
    if (auto sample = steer_delay.poll(now_ns)) {
      steer_meas_deg = *sample;
      has_steer_meas = true;
    }

    std::array<float, 4> wheel_rpm_sample{wheel_info.lf_speed, wheel_info.rf_speed,
                                          wheel_info.lb_speed, wheel_info.rb_speed};
    for (float& rpm : wheel_rpm_sample) {
      rpm = static_cast<float>(rpm + sample_noise(sensor_cfg.wheel_rpm.noise_std));
      if (!std::isfinite(rpm) || rpm < 0.0f) {
        rpm = 0.0f;
      }
    }
    wheel_delay.push(now_ns, wheel_rpm_sample);
    if (auto sample = wheel_delay.poll(now_ns)) {
      wheel_meas_rpm = *sample;
      has_wheel_meas = true;
    }

    TorqueSample torque_sample{};
    torque_sample.front_nm = static_cast<float>((pt_status.front_drive_force - pt_status.front_regen_force) *
                                               wheel_radius +
                                               sample_noise(sensor_cfg.drive_torque.front_noise_std_nm));
    torque_sample.rear_nm = static_cast<float>((pt_status.rear_drive_force - pt_status.rear_regen_force) *
                                              wheel_radius +
                                              sample_noise(sensor_cfg.drive_torque.rear_noise_std_nm));
    torque_delay.push(now_ns, torque_sample);
    if (auto sample = torque_delay.poll(now_ns)) {
      torque_meas = *sample;
      has_torque_meas = true;
    }

    const double max_brake_force = std::max(1.0, model.param().brakes.max_force);
    const double front_max_force = std::max(1e-3, max_brake_force * adapter_cfg.brake_front_bias);
    const double rear_max_force = std::max(1e-3, max_brake_force * adapter_cfg.brake_rear_bias);
    BrakeSample brake_sample{};
    const double front_pct_raw = (std::max(0.0, brake_status.front_force) / front_max_force) * 100.0;
    const double rear_pct_raw = (std::max(0.0, brake_status.rear_force) / rear_max_force) * 100.0;
    brake_sample.front_pct = static_cast<float>(std::clamp(front_pct_raw +
                                           sample_noise(sensor_cfg.brake_pct.noise_std), 0.0, 100.0));
    brake_sample.rear_pct = static_cast<float>(std::clamp(rear_pct_raw +
                                          sample_noise(sensor_cfg.brake_pct.noise_std), 0.0, 100.0));
    brake_sample.front_req_pct = has_last_ai_command ? last_ai2vcu_commands.brake.front_pct : 0.0f;
    brake_sample.rear_req_pct = has_last_ai_command ? last_ai2vcu_commands.brake.rear_pct : 0.0f;
    brake_delay.push(now_ns, brake_sample);
    if (auto sample = brake_delay.poll(now_ns)) {
      brake_meas = *sample;
      has_brake_meas = true;
    }

    speed_delay.push(now_ns, static_cast<float>(std::max(0.0, actual_speed_kph +
                                     sample_noise(sensor_cfg.speed_kph.noise_std))));
    if (auto sample = speed_delay.poll(now_ns)) {
      speed_meas_kph = *sample;
      has_speed_meas = true;
    }

    fsai::sim::svcu::dbc::Ai2LogDynamics2 imu_sample{};
    imu_sample.accel_longitudinal_mps2 = static_cast<float>(
        vehicle_state.acceleration.x() + sample_noise(sensor_cfg.imu.accel_longitudinal_std));
    imu_sample.accel_lateral_mps2 = static_cast<float>(
        vehicle_state.acceleration.y() + sample_noise(sensor_cfg.imu.accel_lateral_std));
    imu_sample.yaw_rate_degps = static_cast<float>(
        vehicle_state.rotation.z() * fsai::sim::svcu::dbc::kRadToDeg +
        sample_noise(sensor_cfg.imu.yaw_rate_std_degps));
    imu_delay.push(now_ns, imu_sample);
    if (auto sample = imu_delay.poll(now_ns)) {
      imu_meas = *sample;
      has_imu_meas = true;
    }

    GpsSample gps_sample{};
    gps_sample.lat_deg = metersToLatitude(vehicle_state.position.y()) +
                         sample_noise(sensor_cfg.gps.lat_std_deg);
    gps_sample.lon_deg = metersToLongitude(vehicle_state.position.x()) +
                         sample_noise(sensor_cfg.gps.lon_std_deg);
    gps_sample.speed_mps = std::max(0.0, vehicle_speed_mps +
                                    sample_noise(sensor_cfg.gps.speed_std_mps));
    gps_delay.push(now_ns, gps_sample);
    if (auto sample = gps_delay.poll(now_ns)) {
      gps_meas = *sample;
      has_gps_meas = true;
    }

    const float steer_for_msg = has_steer_meas ? steer_meas_deg : 0.0f;
    const std::array<float, 4> wheel_for_msg = has_wheel_meas ? wheel_meas_rpm
                                                              : std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f};
    const TorqueSample torque_for_msg = has_torque_meas ? torque_meas : TorqueSample{};
    const BrakeSample brake_for_msg = has_brake_meas ? brake_meas : BrakeSample{};
    const float speed_for_msg_kph = has_speed_meas ? speed_meas_kph : actual_speed_kph;

    fsai::sim::svcu::dbc::Vcu2AiStatus status_msg{};
    status_msg.handshake = true;
    status_msg.as_switch_on = true;
    status_msg.ts_switch_on = true;
    status_msg.go_signal = true;
    status_msg.as_state = world.useRacingAlgorithm
                              ? fsai::sim::svcu::dbc::AsState::kDriving
                              : fsai::sim::svcu::dbc::AsState::kReady;
    status_msg.steering_status = fsai::sim::svcu::dbc::SteeringStatus::kActive;

    fsai::sim::svcu::dbc::Vcu2AiSteer steer_msg{};
    steer_msg.angle_deg = steer_for_msg;
    steer_msg.angle_max_deg = fsai::sim::svcu::dbc::kMaxSteerDeg;
    steer_msg.angle_request_deg = has_last_ai_command
                                      ? last_ai2vcu_commands.steer.steer_deg
                                      : steer_for_msg;

    fsai::sim::svcu::dbc::Vcu2AiDrive front_drive_msg{};
    front_drive_msg.axle_trq_nm = torque_for_msg.front_nm;
    front_drive_msg.axle_trq_max_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
    front_drive_msg.axle_trq_request_nm = has_last_ai_command
        ? last_ai2vcu_commands.front_drive.axle_torque_request_nm
        : std::max(0.0f, torque_for_msg.front_nm);

    fsai::sim::svcu::dbc::Vcu2AiDrive rear_drive_msg{};
    rear_drive_msg.axle_trq_nm = torque_for_msg.rear_nm;
    rear_drive_msg.axle_trq_max_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
    rear_drive_msg.axle_trq_request_nm = has_last_ai_command
        ? last_ai2vcu_commands.rear_drive.axle_torque_request_nm
        : std::max(0.0f, torque_for_msg.rear_nm);

    fsai::sim::svcu::dbc::Vcu2AiSpeeds speed_msg{};
    for (size_t i = 0; i < wheel_for_msg.size(); ++i) {
      speed_msg.wheel_rpm[i] = wheel_for_msg[i];
    }

    fsai::sim::svcu::dbc::Vcu2AiBrake brake_msg{};
    brake_msg.front_pct = brake_for_msg.front_pct;
    brake_msg.rear_pct = brake_for_msg.rear_pct;
    brake_msg.front_req_pct = brake_for_msg.front_req_pct;
    brake_msg.rear_req_pct = brake_for_msg.rear_req_pct;
    brake_msg.status_brk = fsai::sim::svcu::dbc::BrakeStatus::kReady;
    brake_msg.status_ebs = fsai::sim::svcu::dbc::EbsStatus::kArmed;

    fsai::sim::svcu::dbc::Vcu2LogDynamics1 dyn_msg{};
    dyn_msg.speed_actual_kph = speed_for_msg_kph;
    dyn_msg.speed_target_kph = has_last_ai_command
                                   ? last_ai2vcu_commands.status.veh_speed_demand_kph
                                   : speed_for_msg_kph;
    dyn_msg.steer_actual_deg = steer_for_msg;
    dyn_msg.steer_target_deg = has_last_ai_command
                                   ? last_ai2vcu_commands.steer.steer_deg
                                   : steer_for_msg;
    const float brake_target_pct = has_last_ai_command
        ? (last_ai2vcu_commands.brake.front_pct + last_ai2vcu_commands.brake.rear_pct) * 0.5f
        : (brake_for_msg.front_req_pct + brake_for_msg.rear_req_pct) * 0.5f;
    dyn_msg.brake_actual_pct = (brake_for_msg.front_pct + brake_for_msg.rear_pct) * 0.5f;
    dyn_msg.brake_target_pct = brake_target_pct;
    const double total_req_torque_nm = has_last_ai_command
        ? static_cast<double>(last_ai2vcu_commands.front_drive.axle_torque_request_nm +
                              last_ai2vcu_commands.rear_drive.axle_torque_request_nm)
        : 0.0;
    const double total_actual_torque_nm = std::abs(static_cast<double>(torque_for_msg.front_nm)) +
                                          std::abs(static_cast<double>(torque_for_msg.rear_nm));
    auto torque_to_pct = [](double torque_nm) {
      return static_cast<float>(std::clamp(torque_nm /
                                           (2.0 * fsai::sim::svcu::dbc::kMaxAxleTorqueNm) * 100.0,
                                           0.0, 255.0));
    };
    dyn_msg.drive_trq_actual_pct = torque_to_pct(total_actual_torque_nm);
    dyn_msg.drive_trq_target_pct = torque_to_pct(total_req_torque_nm);

    auto send_sim_frame = [&](const can_frame& frame) {
      if (!can_interface.SendSimulationFrame(frame)) {
        std::fprintf(stderr, "Failed to send simulation CAN frame id=%u\n", frame.can_id);
      }
    };

    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_status(status_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_steer(steer_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_drive_front(front_drive_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_drive_rear(rear_drive_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_speeds(speed_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_brake(brake_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2log_dynamics1(dyn_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_ai2log_dynamics2(imu_meas));

    if (has_gps_meas) {
      can_interface.SetSimulationGpsSample(gps_meas.lat_deg, gps_meas.lon_deg,
                                          gps_meas.speed_mps);
    }

    can_interface.Poll(now_ns);

    fsai::sim::svcu::TelemetryPacket telemetry{};
    telemetry.t_ns = now_ns;
    telemetry.steer_angle_rad = appliedSteer;
    telemetry.front_axle_torque_nm = static_cast<float>(front_force * wheel_radius);
    telemetry.rear_axle_torque_nm = static_cast<float>(rear_force * wheel_radius);
    telemetry.wheel_speed_rpm[0] = wheel_info.lf_speed;
    telemetry.wheel_speed_rpm[1] = wheel_info.rf_speed;
    telemetry.wheel_speed_rpm[2] = wheel_info.lb_speed;
    telemetry.wheel_speed_rpm[3] = wheel_info.rb_speed;
    telemetry.brake_pressure_front_bar =
        static_cast<float>(std::max(0.0, brake_status.front_force) / 1000.0);
    telemetry.brake_pressure_rear_bar =
        static_cast<float>(std::max(0.0, brake_status.rear_force) / 1000.0);
    telemetry.imu_ax_mps2 = static_cast<float>(vehicle_state.acceleration.x());
    telemetry.imu_ay_mps2 = static_cast<float>(vehicle_state.acceleration.y());
    telemetry.imu_yaw_rate_rps = static_cast<float>(vehicle_state.rotation.z());
    telemetry.gps_lat_deg = static_cast<float>(
        metersToLatitude(vehicle_state.position.y()));
    telemetry.gps_lon_deg = static_cast<float>(
        metersToLongitude(vehicle_state.position.x()));
    telemetry.gps_speed_mps = static_cast<float>(
        std::sqrt(vehicle_state.velocity.x() * vehicle_state.velocity.x() +
                  vehicle_state.velocity.y() * vehicle_state.velocity.y()));
    telemetry.status_flags = static_cast<uint8_t>(world.useRacingAlgorithm ? 0x3 : 0x1);
    telemetry_tx.send(&telemetry, sizeof(telemetry));

    logger.logState(sim_time_s, world.vehicleState());
    logger.logControl(sim_time_s, world.throttleInput, world.steeringAngle);

    if (stereo_source) {
      const auto& transform = world.vehicleTransform();
      stereo_source->setBodyPose(transform.position.x, transform.position.y,
                                 transform.position.z, transform.yaw);

      cone_positions.clear();
      const auto& left_cones = world.leftConePositions();
      const auto& right_cones = world.rightConePositions();
      cone_positions.reserve(left_cones.size() + right_cones.size());
      for (const auto& cone : left_cones) {
        cone_positions.push_back({cone.x, cone.y, cone.z});
      }
      for (const auto& cone : right_cones) {
        cone_positions.push_back({cone.x, cone.y, cone.z});
      }
      stereo_source->setCones(cone_positions);
      const FsaiStereoFrame& frame = stereo_source->capture(now_ns);
      stereo_display.present(frame);
    }

    {
      fsai::time::SimulationStageTimer render_timer("renderer");
      Graphics_Clear(&graphics);
      Graphics_DrawGrid(&graphics, 50);

      const auto& checkpoints = world.checkpointPositionsWorld();
      if (!checkpoints.empty()) {
        SDL_SetRenderDrawColor(graphics.renderer, 200, 0, 200, 255);
        Graphics_DrawFilledCircle(
            &graphics,
            static_cast<int>(checkpoints.front().x * kRenderScale +
                             graphics.width / 2.0f),
            static_cast<int>(checkpoints.front().z * kRenderScale +
                             graphics.height / 2.0f),
            static_cast<int>(1.5f * kRenderScale));
      }

      const auto& lookahead = world.lookahead();
      SDL_SetRenderDrawColor(graphics.renderer, 120, 120, 120, 255);
      for (size_t i = 0; i < world.leftConePositions().size(); ++i) {
        if (i == 0) {
          SDL_SetRenderDrawColor(graphics.renderer, 0, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.speed) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.steer) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 0, 255, 255);
        } else {
          SDL_SetRenderDrawColor(graphics.renderer, 120, 120, 120, 255);
        }
        const auto& cone = world.leftConePositions()[i];
        const int cone_x = static_cast<int>(cone.x * kRenderScale +
                                            graphics.width / 2.0f);
        const int cone_y = static_cast<int>(cone.z * kRenderScale +
                                            graphics.height / 2.0f);
        Graphics_DrawFilledCircle(&graphics, cone_x, cone_y,
                                  static_cast<int>(kRenderScale));
      }

      for (size_t i = 0; i < world.rightConePositions().size(); ++i) {
        if (i == 0) {
          SDL_SetRenderDrawColor(graphics.renderer, 0, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.speed) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.steer) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 0, 255, 255);
        } else {
          SDL_SetRenderDrawColor(graphics.renderer, 80, 80, 80, 255);
        }
        const auto& cone = world.rightConePositions()[i];
        const int cone_x = static_cast<int>(cone.x * kRenderScale +
                                            graphics.width / 2.0f);
        const int cone_y = static_cast<int>(cone.z * kRenderScale +
                                            graphics.height / 2.0f);
        Graphics_DrawFilledCircle(&graphics, cone_x, cone_y,
                                  static_cast<int>(kRenderScale));
      }

      const auto& transform = world.vehicleTransform();
      const float car_screen_x = transform.position.x * kRenderScale +
                                 graphics.width / 2.0f;
      const float car_screen_y = transform.position.z * kRenderScale +
                                 graphics.height / 2.0f;
      const float car_radius = 2.0f * kRenderScale;
      Graphics_DrawCar(&graphics, car_screen_x, car_screen_y, car_radius,
                       transform.yaw);
      Graphics_Present(&graphics);
    }

    SDL_Delay(static_cast<Uint32>(step_seconds * 1000.0));

    frame_counter++;
    if (frame_counter % kReportIntervalFrames == 0) {
      fsai_budget_report_all();
    }
  }

  fsai_budget_report_all();
  Graphics_Cleanup(&graphics);
  std::printf("Simulation complete. Car state log saved to CarStateLog.csv\n");
  return EXIT_SUCCESS;
}
