#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <queue>
#include <random>
#include <utility>
<<<<<<< HEAD
#include "../../../common/include/common/types.h"
#include "../../sim/svcu/link.hpp"
=======

#include "common/types.h"
#include "link.hpp"
#include "sim/architecture/WorldDebug.hpp"
>>>>>>> dev

namespace fsai::io {

struct TelemetryNoiseConfig {
  double steer_rad_stddev{0.0};
  double axle_torque_stddev{0.0};
  double wheel_rpm_stddev{0.0};
  double brake_bar_stddev{0.0};
  double imu_stddev{0.0};
  double gps_speed_stddev{0.0};
};

class IoBus : public fsai::world::IWorldDebugPublisher {
 public:
  virtual ~IoBus() = default;

  virtual void set_noise_config(const TelemetryNoiseConfig& cfg) = 0;

  virtual void publish_telemetry(const fsai::sim::svcu::TelemetryPacket& raw) = 0;
  virtual std::optional<fsai::sim::svcu::TelemetryPacket> latest_raw_telemetry() = 0;
  virtual std::optional<fsai::sim::svcu::TelemetryPacket> latest_noisy_telemetry() = 0;

  virtual void push_command(const fsai::sim::svcu::CommandPacket& cmd) = 0;
  virtual std::optional<fsai::sim::svcu::CommandPacket> latest_command() = 0;

  virtual void publish_stereo_frame(const FsaiStereoFrame& frame) = 0;
  virtual std::optional<FsaiStereoFrame> latest_stereo_frame() = 0;

  virtual void publish_world_debug(const fsai::world::WorldDebugPacket& packet) = 0;
  virtual std::optional<fsai::world::WorldDebugPacket> latest_world_debug() = 0;

  void publish_debug_packet(const fsai::world::WorldDebugPacket& packet) override {
    publish_world_debug(packet);
  }
};

class InProcessIoBus final : public IoBus {
 public:
  using TelemetrySink = std::function<void(const fsai::sim::svcu::TelemetryPacket&)>;
  using StereoSink = std::function<void(const FsaiStereoFrame&)>;
  using WorldDebugSink =
      std::function<void(const fsai::world::WorldDebugPacket&)>;

  InProcessIoBus();

  void set_noise_config(const TelemetryNoiseConfig& cfg) override;
  void set_telemetry_sink(TelemetrySink sink);
  void set_stereo_sink(StereoSink sink);
  void set_debug_sink(WorldDebugSink sink);

  void publish_telemetry(const fsai::sim::svcu::TelemetryPacket& raw) override;
  std::optional<fsai::sim::svcu::TelemetryPacket> latest_raw_telemetry() override;
  std::optional<fsai::sim::svcu::TelemetryPacket> latest_noisy_telemetry() override;

  void push_command(const fsai::sim::svcu::CommandPacket& cmd) override;
  std::optional<fsai::sim::svcu::CommandPacket> latest_command() override;

  void publish_stereo_frame(const FsaiStereoFrame& frame) override;
  std::optional<FsaiStereoFrame> latest_stereo_frame() override;

  void publish_world_debug(const fsai::world::WorldDebugPacket& packet) override;
  std::optional<fsai::world::WorldDebugPacket> latest_world_debug() override;
  void clear_state();

 private:
  fsai::sim::svcu::TelemetryPacket apply_noise(
      const fsai::sim::svcu::TelemetryPacket& raw);

  TelemetryNoiseConfig noise_{};
  TelemetrySink telemetry_sink_{};
  StereoSink stereo_sink_{};
  WorldDebugSink debug_sink_{};

  std::optional<fsai::sim::svcu::TelemetryPacket> raw_telemetry_{};
  std::optional<fsai::sim::svcu::TelemetryPacket> noisy_telemetry_{};
  std::optional<fsai::sim::svcu::CommandPacket> latest_command_{};
  std::optional<FsaiStereoFrame> last_frame_{};
  std::optional<fsai::world::WorldDebugPacket> last_debug_{};
  std::mt19937 rng_;
};

}  // namespace fsai::io

