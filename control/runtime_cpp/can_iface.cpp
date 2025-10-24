#include "can_iface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>

#if defined(__unix__) || defined(__APPLE__)
#include <dlfcn.h>
#endif

namespace fsai::control::runtime {
namespace {
constexpr float kDegToRad = fsai::sim::svcu::dbc::kDegToRad;
constexpr float kRadToDeg = fsai::sim::svcu::dbc::kRadToDeg;

struct FsAiApiAi2VcuData {
  fsai::sim::svcu::dbc::Ai2VcuStatus status;
  fsai::sim::svcu::dbc::Ai2VcuSteer steer;
  fsai::sim::svcu::dbc::Ai2VcuDrive front_drive;
  fsai::sim::svcu::dbc::Ai2VcuDrive rear_drive;
  fsai::sim::svcu::dbc::Ai2VcuBrake brake;
};

struct FsAiApiVcu2AiData {
  fsai::sim::svcu::dbc::Vcu2AiStatus status;
  fsai::sim::svcu::dbc::Vcu2AiSteer steer;
  fsai::sim::svcu::dbc::Vcu2AiDrive front_drive;
  fsai::sim::svcu::dbc::Vcu2AiDrive rear_drive;
  fsai::sim::svcu::dbc::Vcu2AiBrake brake;
  fsai::sim::svcu::dbc::Vcu2AiSpeeds speeds;
  fsai::sim::svcu::dbc::Vcu2LogDynamics1 dynamics;
};

struct FsAiApiImuData {
  float accel_longitudinal_mps2;
  float accel_lateral_mps2;
  float yaw_rate_degps;
};

struct FsAiApiGpsData {
  double lat_deg;
  double lon_deg;
  double speed_mps;
};
}  // namespace

struct CanIface::FsAiApiVtable {
#if defined(__unix__) || defined(__APPLE__)
  void* handle{nullptr};
#endif
  int (*init_fn)(const char*){nullptr};
  int (*set_fn)(const FsAiApiAi2VcuData*){nullptr};
  int (*get_fn)(FsAiApiVcu2AiData*){nullptr};
  int (*imu_fn)(FsAiApiImuData*){nullptr};
  int (*gps_fn)(FsAiApiGpsData*){nullptr};
};

CanIface::CanIface() = default;
CanIface::~CanIface() { Shutdown(); }

bool CanIface::Initialize(const Config& config) {
  Shutdown();
  mode_ = config.mode;
  initialized_ = false;
  wheel_radius_m_ = std::max(0.01, config.wheel_radius_m);

  switch (mode_) {
    case Mode::kSimulation:
      if (!InitializeSimulation(config)) {
        return false;
      }
      break;
    case Mode::kFsAiApi:
      if (!InitializeFsAiApi(config)) {
        return false;
      }
      break;
  }

  initialized_ = true;
  return true;
}

void CanIface::Shutdown() {
  if (!initialized_) {
    return;
  }

  if (mode_ == Mode::kSimulation && sim_link_) {
    sim_link_->close();
    sim_link_.reset();
  }

#if defined(__unix__) || defined(__APPLE__)
  if (api_) {
    if (api_->handle) {
      dlclose(api_->handle);
    }
  }
#endif
  api_.reset();
  initialized_ = false;
  has_status_ = false;
  has_dyn_ = false;
  has_imu_ = false;
  has_speeds_ = false;
  has_gps_ = false;
}

bool CanIface::Send(const Ai2VcuCommandSet& commands) {
  if (!initialized_) {
    return false;
  }

  if (mode_ == Mode::kSimulation) {
    if (!sim_link_) {
      return false;
    }
    sim_link_->send(fsai::sim::svcu::dbc::encode_ai2vcu_status(commands.status));
    sim_link_->send(fsai::sim::svcu::dbc::encode_ai2vcu_steer(commands.steer));
    sim_link_->send(fsai::sim::svcu::dbc::encode_ai2vcu_drive_front(commands.front_drive));
    sim_link_->send(fsai::sim::svcu::dbc::encode_ai2vcu_drive_rear(commands.rear_drive));
    sim_link_->send(fsai::sim::svcu::dbc::encode_ai2vcu_brake(commands.brake));
    return true;
  }

  if (!api_) {
    return false;
  }

  if (!api_->set_fn) {
    return false;
  }
  FsAiApiAi2VcuData payload{};
  payload.status = commands.status;
  payload.steer = commands.steer;
  payload.front_drive = commands.front_drive;
  payload.rear_drive = commands.rear_drive;
  payload.brake = commands.brake;
  return api_->set_fn(&payload) == 0;
}

void CanIface::Poll(uint64_t now_ns) {
  if (!initialized_) {
    return;
  }
  last_feedback_ns_ = now_ns;
  if (mode_ == Mode::kSimulation) {
    PollSimulation();
  } else {
    PollFsAiApi();
  }
}

bool CanIface::SendSimulationFrame(const can_frame& frame) {
  if (!initialized_ || mode_ != Mode::kSimulation || !sim_link_) {
    return false;
  }
  return sim_link_->send(frame);
}

void CanIface::SetSimulationGpsSample(double lat_deg, double lon_deg, double speed_mps) {
  gps_lat_deg_ = lat_deg;
  gps_lon_deg_ = lon_deg;
  gps_speed_mps_ = speed_mps;
  has_gps_ = true;
}

std::optional<fsai::types::VehicleState> CanIface::LatestVehicleState() const {
  if (!initialized_ || !has_dyn_) {
    return std::nullopt;
  }
  fsai::types::VehicleState state{};
  state.t_ns = last_feedback_ns_;
  state.x = 0.0f;
  state.y = 0.0f;
  state.yaw = 0.0f;
  double speed_mps = 0.0;
  if (has_speeds_) {
    double rpm_sum = 0.0;
    for (float rpm : feedback_speeds_.wheel_rpm) {
      rpm_sum += rpm;
    }
    const double rpm_mean = rpm_sum / 4.0;
    speed_mps = (rpm_mean * 2.0 * std::numbers::pi_v<double> * wheel_radius_m_) / 60.0;
  } else if (has_gps_) {
    speed_mps = gps_speed_mps_;
  } else {
    speed_mps = feedback_dyn_.speed_actual_kph / 3.6f;
  }
  state.vx = static_cast<float>(speed_mps);
  state.vy = 0.0f;
  state.yaw_rate = has_imu_ ? feedback_imu_.yaw_rate_degps * kDegToRad : 0.0f;
  state.ax = has_imu_ ? feedback_imu_.accel_longitudinal_mps2 : 0.0f;
  state.ay = has_imu_ ? feedback_imu_.accel_lateral_mps2 : 0.0f;
  state.steer_rad = feedback_steer_.angle_deg * kDegToRad;
  return state;
}

std::optional<ImuSample> CanIface::LatestImu() const {
  if (!initialized_ || !has_imu_) {
    return std::nullopt;
  }
  ImuSample imu{};
  imu.accel_longitudinal_mps2 = feedback_imu_.accel_longitudinal_mps2;
  imu.accel_lateral_mps2 = feedback_imu_.accel_lateral_mps2;
  imu.yaw_rate_rps = feedback_imu_.yaw_rate_degps * kDegToRad;
  return imu;
}

std::optional<GpsSample> CanIface::LatestGps() const {
  if (!initialized_ || (!has_dyn_ && !has_gps_)) {
    return std::nullopt;
  }
  GpsSample gps{};
  gps.lat_deg = has_gps_ ? gps_lat_deg_ : 0.0;
  gps.lon_deg = has_gps_ ? gps_lon_deg_ : 0.0;
  gps.speed_mps = has_gps_ ? gps_speed_mps_ : feedback_dyn_.speed_actual_kph / 3.6f;
  return gps;
}

bool CanIface::InitializeSimulation(const Config& config) {
  auto link = fsai::sim::svcu::make_can_link(config.endpoint);
  if (!link) {
    return false;
  }
  if (!link->open(config.endpoint, config.enable_loopback)) {
    return false;
  }
  sim_link_ = std::move(link);
  has_status_ = false;
  has_dyn_ = false;
  has_imu_ = false;
  has_speeds_ = false;
  has_gps_ = false;
  return true;
}

bool CanIface::InitializeFsAiApi(const Config& config) {
#if defined(__unix__) || defined(__APPLE__)
  api_ = std::make_unique<FsAiApiVtable>();
  const std::string library_path = !config.api_library.empty() ? config.api_library
                                                              : "libfs_ai_api.so";
  api_->handle = dlopen(library_path.c_str(), RTLD_LAZY);
  if (!api_->handle) {
    return false;
  }
  api_->init_fn = reinterpret_cast<int (*)(const char*)>(dlsym(api_->handle, "fs_ai_api_init"));
  api_->set_fn = reinterpret_cast<int (*)(const FsAiApiAi2VcuData*)>(
      dlsym(api_->handle, "fs_ai_api_ai2vcu_set_data"));
  api_->get_fn = reinterpret_cast<int (*)(FsAiApiVcu2AiData*)>(
      dlsym(api_->handle, "fs_ai_api_vcu2ai_get_data"));
  api_->imu_fn = reinterpret_cast<int (*)(FsAiApiImuData*)>(
      dlsym(api_->handle, "fs_ai_api_imu_get_data"));
  api_->gps_fn = reinterpret_cast<int (*)(FsAiApiGpsData*)>(
      dlsym(api_->handle, "fs_ai_api_gps_get_data"));
  if (!api_->init_fn || !api_->set_fn || !api_->get_fn) {
    return false;
  }
  if (api_->init_fn(config.endpoint.c_str()) != 0) {
    return false;
  }
  return true;
#else
  (void)config;
  return false;
#endif
}

void CanIface::PollSimulation() {
  if (!sim_link_) {
    return;
  }
  while (auto frame = sim_link_->receive()) {
    ProcessFrame(*frame);
  }
}

void CanIface::ProcessFrame(const can_frame& frame) {
  fsai::sim::svcu::dbc::Vcu2AiStatus status{};
  fsai::sim::svcu::dbc::Vcu2AiSteer steer{};
  fsai::sim::svcu::dbc::Vcu2AiDrive drive{};
  fsai::sim::svcu::dbc::Vcu2AiBrake brake{};
  fsai::sim::svcu::dbc::Vcu2LogDynamics1 dyn{};
  fsai::sim::svcu::dbc::Ai2LogDynamics2 imu{};
  fsai::sim::svcu::dbc::Vcu2AiSpeeds speeds{};
  if (fsai::sim::svcu::dbc::decode_vcu2ai_status(frame, status)) {
    feedback_status_ = status;
    has_status_ = true;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_vcu2ai_steer(frame, steer)) {
    feedback_steer_ = steer;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_vcu2ai_drive_front(frame, drive)) {
    feedback_front_drive_ = drive;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_vcu2ai_drive_rear(frame, drive)) {
    feedback_rear_drive_ = drive;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_vcu2ai_brake(frame, brake)) {
    feedback_brake_ = brake;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_vcu2ai_speeds(frame, speeds)) {
    feedback_speeds_ = speeds;
    has_speeds_ = true;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_vcu2log_dynamics1(frame, dyn)) {
    feedback_dyn_ = dyn;
    gps_speed_mps_ = dyn.speed_actual_kph / 3.6f;
    has_dyn_ = true;
    return;
  }
  if (fsai::sim::svcu::dbc::decode_ai2log_dynamics2(frame, imu)) {
    feedback_imu_ = imu;
    has_imu_ = true;
    return;
  }
}

void CanIface::PollFsAiApi() {
  if (!api_ || !api_->get_fn) {
    return;
  }
  FsAiApiVcu2AiData feedback{};
  if (api_->get_fn(&feedback) == 0) {
    feedback_status_ = feedback.status;
    feedback_steer_ = feedback.steer;
    feedback_front_drive_ = feedback.front_drive;
    feedback_rear_drive_ = feedback.rear_drive;
    feedback_brake_ = feedback.brake;
    feedback_dyn_ = feedback.dynamics;
    has_status_ = true;
    has_dyn_ = true;
  }
  if (api_->imu_fn) {
    FsAiApiImuData imu{};
    if (api_->imu_fn(&imu) == 0) {
      feedback_imu_.accel_longitudinal_mps2 = imu.accel_longitudinal_mps2;
      feedback_imu_.accel_lateral_mps2 = imu.accel_lateral_mps2;
      feedback_imu_.yaw_rate_degps = imu.yaw_rate_degps;
      has_imu_ = true;
    }
  }
  if (api_->gps_fn) {
    FsAiApiGpsData gps{};
    if (api_->gps_fn(&gps) == 0) {
      feedback_dyn_.speed_actual_kph = static_cast<float>(std::max(0.0, gps.speed_mps) * 3.6);
      has_dyn_ = true;
    }
  }
}

}  // namespace fsai::control::runtime
