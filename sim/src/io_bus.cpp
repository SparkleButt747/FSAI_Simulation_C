#include "io_bus.hpp"

#include <cmath>

namespace fsai::io {
namespace {

double sample_noise(std::mt19937& rng, double stddev) {
  if (stddev <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> dist(0.0, stddev);
  return dist(rng);
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

fsai::sim::svcu::TelemetryPacket InProcessIoBus::apply_noise(
    const fsai::sim::svcu::TelemetryPacket& raw) {
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
  raw_telemetry_ = raw;
  noisy_telemetry_ = apply_noise(raw);
  if (telemetry_sink_) {
    telemetry_sink_(*noisy_telemetry_);
  }
}

std::optional<fsai::sim::svcu::TelemetryPacket> InProcessIoBus::latest_raw_telemetry() {
  return raw_telemetry_;
}

std::optional<fsai::sim::svcu::TelemetryPacket> InProcessIoBus::latest_noisy_telemetry() {
  return noisy_telemetry_;
}

void InProcessIoBus::push_command(const fsai::sim::svcu::CommandPacket& cmd) {
  latest_command_ = cmd;
}

std::optional<fsai::sim::svcu::CommandPacket> InProcessIoBus::latest_command() {
  return latest_command_;
}

void InProcessIoBus::publish_stereo_frame(const FsaiStereoFrame& frame) {
  last_frame_ = frame;
  if (stereo_sink_) {
    stereo_sink_(frame);
  }
}

std::optional<FsaiStereoFrame> InProcessIoBus::latest_stereo_frame() { return last_frame_; }

void InProcessIoBus::publish_world_debug(
    const fsai::world::WorldDebugPacket& packet) {
  last_debug_ = packet;
  if (debug_sink_) {
    debug_sink_(packet);
  }
}

std::optional<fsai::world::WorldDebugPacket> InProcessIoBus::latest_world_debug() {
  return last_debug_;
}

void InProcessIoBus::clear_state() {
  raw_telemetry_.reset();
  noisy_telemetry_.reset();
  latest_command_.reset();
  last_frame_.reset();
  last_debug_.reset();
}

}  // namespace fsai::io

