#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "MissionDefinition.hpp"
#include "MissionRuntimeState.hpp"
#include "Transform.h"
#include "Vector.h"

namespace fsai::sim {

class WorldRuntime {
 public:
  enum class ResetReason {
    kConeCollision,
    kBoundaryCollision,
    kTrackRegeneration,
    kExternal,
    kUnknown,
  };

  struct ResetEvent {
    ResetReason reason{ResetReason::kUnknown};
  };

  struct Config {
    float lap_completion_threshold{0.1f};
  };

  struct LapEvent {
    double lap_time_s{0.0};
    double lap_distance_m{0.0};
    int lap_index{0};
  };

  using ResetCallback = std::function<void(const ResetEvent&)>;
  using MissionCompleteCallback =
      std::function<void(const MissionRuntimeState&)>;

  WorldRuntime() = default;

  void Configure(const MissionDefinition& mission, const Config& config);
  void UpdateTrackContext(const std::vector<Vector3>& checkpoints);
  void NotifySpawnApplied(const Transform& transform);
  void BeginStep(double dt_seconds, const VehicleState& vehicle_state);
  void AccumulateDistance(double delta_distance_m);
  std::optional<LapEvent> RegisterGateCrossing();
  void UpdateStraightLineProgress(const Transform& transform);
  void HandleMissionCompletion();

  void ResetMission();

  void AddResetListener(ResetCallback callback);
  void AddMissionCompleteListener(MissionCompleteCallback callback);
  void EmitResetEvent(ResetReason reason);
  void MarkResetPending(ResetReason reason);
  void AcknowledgeResetRequest();

  void MarkMissionCompleted();

  const MissionRuntimeState& mission_state() const { return mission_state_; }
  double lap_time_seconds() const { return lap_time_s_; }
  double lap_distance_meters() const { return lap_distance_m_; }
  double time_step_seconds() const { return delta_time_s_; }
  int lap_count() const { return lap_count_; }
  std::optional<ResetReason> pending_reset_reason() const {
    return pending_reset_reason_;
  }

 private:
  void ConfigureStraightLineTracker();

  MissionDefinition mission_{};
  MissionRuntimeState mission_state_{};
  Config config_{};

  std::vector<Vector3> checkpoints_snapshot_{};
  std::size_t checkpoints_per_lap_{0};
  std::size_t next_checkpoint_index_{0};
  std::size_t gates_since_last_lap_{0};
  bool start_gate_seen_{false};

  double lap_time_s_{0.0};
  double lap_distance_m_{0.0};
  double delta_time_s_{0.0};
  int lap_count_{0};
  bool mission_complete_notified_{false};
  std::optional<ResetReason> pending_reset_reason_{};

  struct StraightLineTracker {
    bool valid{false};
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    Eigen::Vector2d direction{Eigen::Vector2d::UnitX()};
    double length{0.0};
  } straight_tracker_{};

  std::vector<ResetCallback> reset_listeners_{};
  std::vector<MissionCompleteCallback> mission_complete_listeners_{};
};

}  // namespace fsai::sim
