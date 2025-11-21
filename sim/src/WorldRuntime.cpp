#include "sim/WorldRuntime.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::sim {

void WorldRuntime::Configure(const MissionDefinition& mission,
                             const Config& config) {
  mission_ = mission;
  config_ = config;
  ResetMission();
}

void WorldRuntime::UpdateTrackContext(const std::vector<Vector3>& checkpoints) {
  checkpoints_snapshot_ = checkpoints;
  if (!checkpoints_snapshot_.empty()) {
    last_checkpoint_ = checkpoints_snapshot_.back();
  } else {
    last_checkpoint_ = Vector3{0.0f, 0.0f, 0.0f};
  }
  ConfigureStraightLineTracker();
}

void WorldRuntime::NotifySpawnApplied(const Transform& transform) {
  const float distance = DistanceToLastCheckpoint(transform);
  inside_last_checkpoint_ =
      distance < config_.lap_completion_threshold;
}

void WorldRuntime::BeginStep(double dt_seconds, const VehicleState& vehicle_state) {
  delta_time_s_ = dt_seconds;
  if (dt_seconds <= 0.0) {
    return;
  }
  mission_state_.Update(dt_seconds, vehicle_state);
  if (!mission_state_.mission_complete()) {
    lap_time_s_ += dt_seconds;
  }
}

void WorldRuntime::AccumulateDistance(double delta_distance_m) {
  if (mission_state_.mission_complete()) {
    return;
  }
  lap_distance_m_ += delta_distance_m;
}

std::optional<WorldRuntime::LapEvent> WorldRuntime::EvaluateLapTransition(
    const Transform& transform) {
  const float distance = DistanceToLastCheckpoint(transform);
  const bool inside_now = distance < config_.lap_completion_threshold;
  std::optional<LapEvent> event;
  if (inside_now && !inside_last_checkpoint_ &&
      !mission_state_.mission_complete()) {
    const double completed_time = lap_time_s_;
    const double completed_distance = lap_distance_m_;
    mission_state_.RegisterLap(completed_time, completed_distance);
    lap_count_ = static_cast<int>(mission_state_.completed_laps());
    event = LapEvent{completed_time, completed_distance, lap_count_};
    lap_time_s_ = 0.0;
    lap_distance_m_ = 0.0;
  }
  inside_last_checkpoint_ = inside_now;
  return event;
}

void WorldRuntime::UpdateStraightLineProgress(const Transform& transform) {
  if (!straight_tracker_.valid) {
    return;
  }
  const Eigen::Vector2d position(
      static_cast<double>(transform.position.x),
      static_cast<double>(transform.position.z));
  const Eigen::Vector2d offset = position - straight_tracker_.origin;
  const double projection = offset.dot(straight_tracker_.direction);
  const double clamped =
      std::clamp(projection, 0.0, straight_tracker_.length);
  mission_state_.SetStraightLineProgress(clamped);
}

void WorldRuntime::HandleMissionCompletion() {
  if (!mission_state_.mission_complete() || mission_complete_notified_) {
    return;
  }
  if (!mission_state_.stop_commanded()) {
    mission_state_.MarkStopCommanded();
  }
  mission_complete_notified_ = true;
  for (const auto& listener : mission_complete_listeners_) {
    if (listener) {
      listener(mission_state_);
    }
  }
}

void WorldRuntime::ResetMission() {
  mission_state_.Reset(mission_);
  lap_time_s_ = 0.0;
  lap_distance_m_ = 0.0;
  delta_time_s_ = 0.0;
  lap_count_ = 0;
  inside_last_checkpoint_ = false;
  mission_complete_notified_ = false;
}

void WorldRuntime::AddResetListener(ResetCallback callback) {
  if (callback) {
    reset_listeners_.push_back(callback);
  }
}

void WorldRuntime::AddMissionCompleteListener(
    MissionCompleteCallback callback) {
  if (callback) {
    mission_complete_listeners_.push_back(callback);
  }
}

void WorldRuntime::EmitResetEvent(ResetReason reason) {
  MarkResetPending(reason);
  const ResetEvent event{reason};
  for (const auto& listener : reset_listeners_) {
    if (listener) {
      listener(event);
    }
  }
}

void WorldRuntime::MarkResetPending(ResetReason reason) {
  pending_reset_reason_ = reason;
}

void WorldRuntime::AcknowledgeResetRequest() { pending_reset_reason_.reset(); }

void WorldRuntime::ConfigureStraightLineTracker() {
  straight_tracker_ = {};
  if (mission_.descriptor.type != MissionType::kAcceleration) {
    return;
  }
  if (checkpoints_snapshot_.size() < 2) {
    return;
  }
  const Vector3& start = checkpoints_snapshot_.front();
  const Vector3& finish = checkpoints_snapshot_.back();
  const Eigen::Vector2d start2(static_cast<double>(start.x),
                               static_cast<double>(start.z));
  const Eigen::Vector2d finish2(static_cast<double>(finish.x),
                                static_cast<double>(finish.z));
  const Eigen::Vector2d delta = finish2 - start2;
  const double length = delta.norm();
  if (length <= 1e-3) {
    return;
  }
  straight_tracker_.valid = true;
  straight_tracker_.origin = start2;
  straight_tracker_.direction = delta / length;
  straight_tracker_.length = length;
}

float WorldRuntime::DistanceToLastCheckpoint(
    const Transform& transform) const {
  const float dx = transform.position.x - last_checkpoint_.x;
  const float dz = transform.position.z - last_checkpoint_.z;
  return std::sqrt(dx * dx + dz * dz);
}

}  // namespace fsai::sim

