#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "ai2vcu_adapter.hpp"
#include "adsdv_dbc.hpp"
#include "io/can/can_transport.hpp"
#include "types.h"

namespace fsai::control::runtime {

struct ImuSample {
  float accel_longitudinal_mps2{0.0f};
  float accel_lateral_mps2{0.0f};
  float yaw_rate_rps{0.0f};
};

struct GpsSample {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double speed_mps{0.0};
};

class CanIface {
 public:
  enum class Mode { kSimulation, kFsAiApi };

  struct Config {
    Mode mode{Mode::kSimulation};
    std::string endpoint;
    bool enable_loopback{true};
    std::string api_library;
    double wheel_radius_m{0.25};
  };

  CanIface();
  ~CanIface();

  bool Initialize(const Config& config);
  void Shutdown();

  bool Send(const Ai2VcuCommandSet& commands, uint64_t now_ns = 0);
  void Poll(uint64_t now_ns);

  bool SendSimulationFrame(const can_frame& frame);
  void SetSimulationGpsSample(double lat_deg, double lon_deg, double speed_mps);

  std::optional<fsai::types::VehicleState> LatestVehicleState() const;
  std::optional<ImuSample> LatestImu() const;
  std::optional<GpsSample> LatestGps() const;

  Mode mode() const { return mode_; }
  const std::string& endpoint() const { return endpoint_; }

  bool HasStatus() const { return has_status_; }
  bool HasSteer() const { return has_steer_; }
  bool HasFrontDrive() const { return has_front_drive_; }
  bool HasRearDrive() const { return has_rear_drive_; }
  bool HasBrake() const { return has_brake_; }
  bool HasSpeeds() const { return has_speeds_; }
  bool HasDynamics() const { return has_dyn_; }

  uint64_t LastStatusTimestampNs() const { return last_status_ns_; }
  uint64_t LastSteerTimestampNs() const { return last_steer_ns_; }
  uint64_t LastFrontDriveTimestampNs() const { return last_front_drive_ns_; }
  uint64_t LastRearDriveTimestampNs() const { return last_rear_drive_ns_; }
  uint64_t LastBrakeTimestampNs() const { return last_brake_ns_; }
  uint64_t LastSpeedsTimestampNs() const { return last_speeds_ns_; }
  uint64_t LastDynamicsTimestampNs() const { return last_dyn_ns_; }

  const fsai::sim::svcu::dbc::Vcu2AiStatus& RawStatus() const { return feedback_status_; }
  const fsai::sim::svcu::dbc::Vcu2AiSteer& RawSteer() const { return feedback_steer_; }
  const fsai::sim::svcu::dbc::Vcu2AiDrive& RawFrontDrive() const { return feedback_front_drive_; }
  const fsai::sim::svcu::dbc::Vcu2AiDrive& RawRearDrive() const { return feedback_rear_drive_; }
  const fsai::sim::svcu::dbc::Vcu2AiBrake& RawBrake() const { return feedback_brake_; }
  const fsai::sim::svcu::dbc::Vcu2LogDynamics1& RawDynamics() const { return feedback_dyn_; }
  const fsai::sim::svcu::dbc::Vcu2AiSpeeds& RawSpeeds() const { return feedback_speeds_; }

 private:
  bool InitializeSimulation(const Config& config);
  bool InitializeFsAiApi(const Config& config);
  bool TransmitFsAiApi(const Ai2VcuCommandSet& commands);
  void PollSimulation(uint64_t now_ns);
  void PollFsAiApi(uint64_t now_ns);
  void ProcessFrame(const can_frame& frame, uint64_t timestamp_ns);

  Mode mode_{Mode::kSimulation};
  bool initialized_{false};
  std::string endpoint_{};
  std::unique_ptr<fsai::io::can::ICanTransport> transport_;
  double wheel_radius_m_{0.25};

  fsai::sim::svcu::dbc::Vcu2AiStatus feedback_status_{};
  fsai::sim::svcu::dbc::Vcu2AiSteer feedback_steer_{};
  fsai::sim::svcu::dbc::Vcu2AiDrive feedback_front_drive_{};
  fsai::sim::svcu::dbc::Vcu2AiDrive feedback_rear_drive_{};
  fsai::sim::svcu::dbc::Vcu2AiBrake feedback_brake_{};
  fsai::sim::svcu::dbc::Vcu2LogDynamics1 feedback_dyn_{};
  fsai::sim::svcu::dbc::Ai2LogDynamics2 feedback_imu_{};
  fsai::sim::svcu::dbc::Vcu2AiSpeeds feedback_speeds_{};
  uint64_t last_feedback_ns_{0};
  uint64_t current_poll_ns_{0};
  uint64_t last_status_ns_{0};
  uint64_t last_steer_ns_{0};
  uint64_t last_front_drive_ns_{0};
  uint64_t last_rear_drive_ns_{0};
  uint64_t last_brake_ns_{0};
  uint64_t last_speeds_ns_{0};
  uint64_t last_dyn_ns_{0};
  bool has_steer_{false};
  bool has_front_drive_{false};
  bool has_rear_drive_{false};
  bool has_brake_{false};
  bool has_status_{false};
  bool has_dyn_{false};
  bool has_imu_{false};
  bool has_speeds_{false};
  double gps_lat_deg_{0.0};
  double gps_lon_deg_{0.0};
  double gps_speed_mps_{0.0};
  bool has_gps_{false};

  struct FsAiApiVtable;
  std::unique_ptr<FsAiApiVtable> api_;
  std::optional<Ai2VcuCommandSet> last_commands_{};
  bool pending_api_tx_{false};
  uint64_t last_api_tx_ns_{0};
  static constexpr uint64_t kAi2VcuPeriodNs = 10'000'000;  // 10 ms
};

}  // namespace fsai::control::runtime
