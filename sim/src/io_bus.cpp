#include "io_bus.hpp"

#include <cmath>
#include <limits>

#include "fsai_clock.h"
#include "../include/logging.hpp"

// IO SUBSYSTEM: InProcessIoBus glues simulated telemetry, commands, and stereo
// frames together inside a single process. The agent publishes ground-truth
// telemetry from Vehicle Dynamics into this bus, optionally adds noise, then
// fans it out to the Control and Vision frontends via callbacks or buffered
// reads. The same object also carries Control commands back into the sim and
// Vision stereo frames into the Vision pipeline.

namespace fsai::io {
namespace {

double sample_noise(std::mt19937& rng, double stddev) {
  if (stddev <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> dist(0.0, stddev);
  return dist(rng);
}

double age_seconds(uint64_t now_ns, uint64_t t_ns) {
  if (t_ns == 0 || now_ns == 0 || now_ns < t_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - t_ns) * 1e-9;
}

bool telemetry_finite(const fsai::sim::svcu::TelemetryPacket& pkt) {
  if (!std::isfinite(pkt.steer_angle_rad) || !std::isfinite(pkt.front_axle_torque_nm) ||
      !std::isfinite(pkt.rear_axle_torque_nm) || !std::isfinite(pkt.brake_pressure_front_bar) ||
      !std::isfinite(pkt.brake_pressure_rear_bar) || !std::isfinite(pkt.imu_ax_mps2) ||
      !std::isfinite(pkt.imu_ay_mps2) || !std::isfinite(pkt.imu_yaw_rate_rps) ||
      !std::isfinite(pkt.gps_speed_mps)) {
    return false;
  }
  for (float rpm : pkt.wheel_speed_rpm) {
    if (!std::isfinite(rpm)) {
      return false;
    }
  }
  return true;
}

bool command_finite(const fsai::sim::svcu::CommandPacket& cmd) {
  return std::isfinite(cmd.steer_rad) && std::isfinite(cmd.throttle) &&
         std::isfinite(cmd.brake);
}

}  // namespace

InProcessIoBus::InProcessIoBus() : rng_(std::random_device{}()) {}

void InProcessIoBus::set_noise_config(const TelemetryNoiseConfig& cfg) {
  noise_ = cfg;
}

void InProcessIoBus::set_telemetry_sink(TelemetrySink sink) {
  telemetry_sink_ = std::move(sink);
}

void InProcessIoBus::set_stereo_sink(StereoSink sink) { stereo_sink_ = std::move(sink); }

void InProcessIoBus::set_debug_sink(WorldDebugSink sink) {
  debug_sink_ = std::move(sink);
}

void InProcessIoBus::set_health_sink(std::function<void(const Health&)> sink) {
  health_sink_ = std::move(sink);
}

void InProcessIoBus::set_staleness_thresholds(double command_age_s, double telemetry_age_s) {
  command_stale_threshold_s_ = command_age_s;
  telemetry_stale_threshold_s_ = telemetry_age_s;
}

fsai::sim::svcu::TelemetryPacket InProcessIoBus::apply_noise(
    const fsai::sim::svcu::TelemetryPacket& raw) {
  // BACKEND NOISE MODEL: perturb ground-truth telemetry before sending it to
  // the Control/Vision consumers to approximate realistic sensors. All values
  // are zeroed today but the plumbing is ready for future tuning.
  fsai::sim::svcu::TelemetryPacket noisy = raw;
  noisy.steer_angle_rad += static_cast<float>(sample_noise(rng_, noise_.steer_rad_stddev));
  noisy.front_axle_torque_nm +=
      static_cast<float>(sample_noise(rng_, noise_.axle_torque_stddev));
  noisy.rear_axle_torque_nm +=
      static_cast<float>(sample_noise(rng_, noise_.axle_torque_stddev));
  for (float& rpm : noisy.wheel_speed_rpm) {
    rpm += static_cast<float>(sample_noise(rng_, noise_.wheel_rpm_stddev));
  }
  noisy.brake_pressure_front_bar +=
      static_cast<float>(sample_noise(rng_, noise_.brake_bar_stddev));
  noisy.brake_pressure_rear_bar +=
      static_cast<float>(sample_noise(rng_, noise_.brake_bar_stddev));
  noisy.imu_ax_mps2 += static_cast<float>(sample_noise(rng_, noise_.imu_stddev));
  noisy.imu_ay_mps2 += static_cast<float>(sample_noise(rng_, noise_.imu_stddev));
  noisy.imu_yaw_rate_rps += static_cast<float>(sample_noise(rng_, noise_.imu_stddev));
  noisy.gps_speed_mps += static_cast<float>(sample_noise(rng_, noise_.gps_speed_stddev));
  return noisy;
}

void InProcessIoBus::publish_telemetry(const fsai::sim::svcu::TelemetryPacket& raw) {
  const uint64_t now_ns = fsai_clock_now();
  const bool was_stale = health_.telemetry_stale;
  if (!telemetry_finite(raw)) {
    health_.last_error = "Telemetry rejected: non-finite values";
    health_.telemetry_valid = false;
    health_.telemetry_stale = true;
    fsai::sim::log::LogError(health_.last_error);
    publish_health();
    return;
  }

  raw_telemetry_ = raw;
  noisy_telemetry_ = apply_noise(raw);
  last_telemetry_t_ns_ = raw.t_ns;

  health_.telemetry_valid = true;
  health_.telemetry_age_s = age_seconds(now_ns, raw.t_ns);
  health_.telemetry_stale = health_.telemetry_age_s > telemetry_stale_threshold_s_;
  health_.last_error.clear();
  if (health_.telemetry_stale && !was_stale) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "Telemetry stale (age %.3f s); holding last good frame",
                         health_.telemetry_age_s);
  }
  publish_health();

  // IO->CONTROL/VISION: push the noisy packet downstream via the optional
  // callback hook (used by Control) while keeping the latest copy for GUI
  // panels and Vision side reads.
  if (telemetry_sink_) {
    telemetry_sink_(*noisy_telemetry_);
  }
}

std::optional<fsai::sim::svcu::TelemetryPacket> InProcessIoBus::latest_raw_telemetry() {
  if (raw_telemetry_) {
    const uint64_t now_ns = fsai_clock_now();
    const bool was_stale = health_.telemetry_stale;
    health_.telemetry_age_s = age_seconds(now_ns, last_telemetry_t_ns_);
    health_.telemetry_stale = health_.telemetry_age_s > telemetry_stale_threshold_s_;
    if (health_.telemetry_stale && !was_stale) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Telemetry stale (age %.3f s); holding last good frame",
                           health_.telemetry_age_s);
    }
    if (health_.telemetry_stale != was_stale) {
      publish_health();
    }
  }
  return raw_telemetry_;
}

std::optional<fsai::sim::svcu::TelemetryPacket> InProcessIoBus::latest_noisy_telemetry() {
  if (noisy_telemetry_) {
    const uint64_t now_ns = fsai_clock_now();
    const bool was_stale = health_.telemetry_stale;
    health_.telemetry_age_s = age_seconds(now_ns, last_telemetry_t_ns_);
    health_.telemetry_stale = health_.telemetry_age_s > telemetry_stale_threshold_s_;
    if (health_.telemetry_stale != was_stale) {
      publish_health();
    }
  }
  return noisy_telemetry_;
}

void InProcessIoBus::push_command(const fsai::sim::svcu::CommandPacket& cmd) {
  // CONTROL->IO->VEHICLE: enqueue the latest actuator command so the vehicle
  // dynamics adapter can pick it up on the next tick.
  const uint64_t now_ns = fsai_clock_now();
  const bool was_stale = health_.command_stale;
  if (!command_finite(cmd)) {
    health_.last_error = "Command rejected: non-finite values";
    health_.command_valid = false;
    health_.command_stale = true;
    fsai::sim::log::LogError(health_.last_error);
    publish_health();
    return;
  }

  latest_command_ = cmd;
  last_command_t_ns_ = cmd.t_ns;
  health_.command_valid = true;
  health_.command_age_s = age_seconds(now_ns, cmd.t_ns);
  health_.command_stale = health_.command_age_s > command_stale_threshold_s_;
  health_.last_error.clear();
  if (health_.command_stale && !was_stale) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "Command stale (age %.3f s); awaiting fresh inputs",
                         health_.command_age_s);
  }
  publish_health();
}

std::optional<fsai::sim::svcu::CommandPacket> InProcessIoBus::latest_command() {
  if (latest_command_) {
    const uint64_t now_ns = fsai_clock_now();
    const bool was_stale = health_.command_stale;
    health_.command_age_s = age_seconds(now_ns, last_command_t_ns_);
    health_.command_stale = health_.command_age_s > command_stale_threshold_s_;
    if (health_.command_stale && !was_stale) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Command stale (age %.3f s); awaiting fresh inputs",
                           health_.command_age_s);
    }
    if (health_.command_stale != was_stale) {
      publish_health();
    }
  }
  return latest_command_;
}

void InProcessIoBus::publish_stereo_frame(const FsaiStereoFrame& frame) {
  last_frame_ = frame;
  // WORLD/GRAPHICS->VISION: forward simulated stereo images into the Vision
  // frame ring buffer or previews to keep the perception pipeline fed.
  if (stereo_sink_) {
    stereo_sink_(frame);
  }
}

std::optional<FsaiStereoFrame> InProcessIoBus::latest_stereo_frame() { return last_frame_; }

void InProcessIoBus::publish_world_debug(
    const fsai::world::WorldDebugPacket& packet) {
  last_debug_ = packet;
  // WORLD->IO->VISION/CONTROL: broadcast debug overlays (ground truth cones,
  // controller path, detections) so both the GUI and downstream systems can
  // visualize shared context.
  if (debug_sink_) {
    debug_sink_(packet);
  }
}

std::optional<fsai::world::WorldDebugPacket> InProcessIoBus::latest_world_debug() {
  return last_debug_;
}

void InProcessIoBus::publish_health() {
  if (health_sink_) {
    health_sink_(health_);
  }
}

void InProcessIoBus::clear_state() {
  raw_telemetry_.reset();
  noisy_telemetry_.reset();
  latest_command_.reset();
  last_frame_.reset();
  last_debug_.reset();
  last_telemetry_t_ns_ = 0;
  last_command_t_ns_ = 0;
  health_ = {};
  publish_health();
}

}  // namespace fsai::io
