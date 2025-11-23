#pragma once

#include <cstdint>
#include <optional>

#include "io_bus.hpp"
#include "sim/svcu/link.hpp"

namespace fsai::world {
class IWorldView;
}

namespace fsai::io {

struct ControlCommand {
  float steer_rad{0.0f};
  float throttle{0.0f};
  float brake{0.0f};
  uint64_t t_ns{0};
};

/**
 * Facade for all external control/vision ingress and telemetry egress.
 * This is the shared conduit between control loops and the simulation core.
 */
class IIoFacade {
 public:
  virtual ~IIoFacade() = default;

  // Control ingress from AI/UI/etc.
  virtual void push_command(const ControlCommand& command) = 0;
  virtual std::optional<ControlCommand> latest_command() const = 0;

  // Ground-truth capture from the world before noise injection.
  virtual void publish_ground_truth(const fsai::world::IWorldView& world) = 0;

  // Telemetry stream (raw + noisy) for control consumers.
  virtual std::optional<fsai::sim::svcu::TelemetryPacket> latest_raw_telemetry() const = 0;
  virtual std::optional<fsai::sim::svcu::TelemetryPacket> latest_noisy_telemetry() const = 0;

  // Stereo imagery for perception; may be ground-truth or noise-augmented.
  virtual void publish_stereo_frame(const FsaiStereoFrame& frame) = 0;
  virtual std::optional<FsaiStereoFrame> latest_stereo_frame() const = 0;

  // Configure how telemetry noise is injected.
  virtual void set_noise_config(const TelemetryNoiseConfig& cfg) = 0;
};

}  // namespace fsai::io

