#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "ai2vcu_adapter.hpp"
#include "adsdv_dbc.hpp"
#include "can_link.hpp"
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

  const fsai::sim::svcu::dbc::Vcu2AiStatus& RawStatus() const { return feedback_status_; }
  const fsai::sim::svcu::dbc::Vcu2AiSteer& RawSteer() const { return feedback_steer_; }
  const fsai::sim::svcu::dbc::Vcu2AiDrive& RawFrontDrive() const { return feedback_front_drive_; }
  const fsai::sim::svcu::dbc::Vcu2AiDrive& RawRearDrive() const { return feedback_rear_drive_; }
  const fsai::sim::svcu::dbc::Vcu2AiBrake& RawBrake() const { return feedback_brake_; }
  const fsai::sim::svcu::dbc::Vcu2LogDynamics1& RawDynamics() const { return feedback_dyn_; }

 private:
  bool InitializeSimulation(const Config& config);
  bool InitializeFsAiApi(const Config& config);
  bool TransmitFsAiApi(const Ai2VcuCommandSet& commands);
  void PollSimulation();
  void PollFsAiApi(uint64_t now_ns);
  void ProcessFrame(const can_frame& frame);

  Mode mode_{Mode::kSimulation};
  bool initialized_{false};
  std::unique_ptr<fsai::sim::svcu::ICanLink> sim_link_;
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
