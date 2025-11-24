#include "sim/WorldRuntime.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::sim {

namespace {

Eigen::Vector2d toVector2d(const Vector3& v) {
  return Eigen::Vector2d(static_cast<double>(v.x), static_cast<double>(v.z));
}

}  // namespace

WorldRuntime::WorldRuntime(MissionRuntimeState& mission_state)
    : mission_state_(mission_state) {}

void WorldRuntime::Configure(const MissionDefinition& mission,
                             const std::vector<Vector3>& checkpoints,
                             const WorldRuntimeConfig& config) {
  mission_state_.Reset(mission);
  config_ = config;
  straight_tracker_ = {};
  configureStraightLineTracker(mission, checkpoints);
  inside_last_checkpoint_ = false;
  resetLapMetrics();
}

void WorldRuntime::InitializeLapTracking(const Transform& spawn_transform,
                                         const Vector3& last_checkpoint) {
  const float dx = spawn_transform.position.x - last_checkpoint.x;
  const float dz = spawn_transform.position.z - last_checkpoint.z;
  const float dist = std::sqrt(dx * dx + dz * dz);
  inside_last_checkpoint_ = dist < config_.lap_completion_threshold;
}

void WorldRuntime::AdvanceMission(double dt_seconds,
                                  const Eigen::Vector2d& velocity_2d) {
  if (mission_state_.mission_complete()) {
    return;
  }

  mission_state_.Update(dt_seconds);
  lap_time_seconds_ += dt_seconds;
  lap_distance_meters_ += velocity_2d.norm() * dt_seconds;
}

std::optional<LapRecord> WorldRuntime::EvaluateLapCrossing(
    const Transform& car_transform, const Vector3& last_checkpoint) {
  const float dx = car_transform.position.x - last_checkpoint.x;
  const float dz = car_transform.position.z - last_checkpoint.z;
  const float dist_to_last = std::sqrt(dx * dx + dz * dz);
  const bool inside_now = dist_to_last < config_.lap_completion_threshold;

  if (inside_now && !inside_last_checkpoint_ && !mission_state_.mission_complete()) {
    const LapRecord record{lap_time_seconds_, lap_distance_meters_};
    mission_state_.RegisterLap(lap_time_seconds_, lap_distance_meters_);
    resetLapMetrics();
    inside_last_checkpoint_ = true;
    return record;
  }

  inside_last_checkpoint_ = inside_now;
  return std::nullopt;
}

void WorldRuntime::UpdateStraightLineProgress(const Transform& car_transform) {
  if (!straight_tracker_.valid) {
    return;
  }

  const Eigen::Vector2d position(static_cast<double>(car_transform.position.x),
                                 static_cast<double>(car_transform.position.z));
  const Eigen::Vector2d offset = position - straight_tracker_.origin;
  const double projection = offset.dot(straight_tracker_.direction);
  const double clamped = std::clamp(projection, 0.0, straight_tracker_.length);
  mission_state_.SetStraightLineProgress(clamped);
}

void WorldRuntime::CheckMissionComplete() {
  if (!mission_state_.mission_complete() || mission_state_.stop_commanded()) {
    return;
  }

  mission_state_.MarkStopCommanded();
  if (events_.on_mission_complete) {
    events_.on_mission_complete();
  }
}

void WorldRuntime::RequestReset() {
  if (events_.on_reset_requested) {
    events_.on_reset_requested();
  }
}

void WorldRuntime::configureStraightLineTracker(
    const MissionDefinition& mission, const std::vector<Vector3>& checkpoints) {
  if (mission.descriptor.type != MissionType::kAcceleration ||
      checkpoints.size() < 2) {
    return;
  }

  const Eigen::Vector2d start = toVector2d(checkpoints.front());
  const Eigen::Vector2d finish = toVector2d(checkpoints.back());
  const Eigen::Vector2d delta = finish - start;
  const double length = delta.norm();
  if (length <= 1e-3) {
    return;
  }

  straight_tracker_.valid = true;
  straight_tracker_.origin = start;
  straight_tracker_.direction = delta / length;
  straight_tracker_.length = length;
}

void WorldRuntime::resetLapMetrics() {
  lap_time_seconds_ = 0.0;
  lap_distance_meters_ = 0.0;
}

}  // namespace fsai::sim

