#pragma once

#include <functional>
#include <optional>
#include <utility>

#include <Eigen/Dense>

#include "Transform.h"
#include "sim/MissionRuntimeState.hpp"
#include "sim/mission/MissionDefinition.hpp"
#include "Vector.h"

namespace fsai::sim {

struct WorldRuntimeConfig {
  float collision_threshold{2.5f};
  float vehicle_collision_radius{0.386f};
  float lap_completion_threshold{0.1f};
};

struct LapRecord {
  double time_seconds{0.0};
  double distance_meters{0.0};
};

class WorldRuntime {
 public:
  struct Events {
    std::function<void()> on_reset_requested{};
    std::function<void()> on_mission_complete{};
  };

  explicit WorldRuntime(MissionRuntimeState& mission_state);

  void Configure(const MissionDefinition& mission,
                 const std::vector<Vector3>& checkpoints,
                 const WorldRuntimeConfig& config);

  void InitializeLapTracking(const Transform& spawn_transform,
                             const Vector3& last_checkpoint);

  void AdvanceMission(double dt_seconds, const Eigen::Vector2d& velocity_2d);

  std::optional<LapRecord> EvaluateLapCrossing(const Transform& car_transform,
                                               const Vector3& last_checkpoint);

  void UpdateStraightLineProgress(const Transform& car_transform);

  void CheckMissionComplete();

  void RequestReset();

  void set_events(const Events& events) { events_ = events; }

  double lap_time_seconds() const { return lap_time_seconds_; }
  double lap_distance_meters() const { return lap_distance_meters_; }
  const WorldRuntimeConfig& config() const { return config_; }

 private:
  struct StraightLineTracker {
    bool valid{false};
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    Eigen::Vector2d direction{Eigen::Vector2d::UnitX()};
    double length{0.0};
  };

  void configureStraightLineTracker(const MissionDefinition& mission,
                                    const std::vector<Vector3>& checkpoints);
  void resetLapMetrics();

  MissionRuntimeState& mission_state_;
  WorldRuntimeConfig config_{};
  Events events_{};
  bool inside_last_checkpoint_{false};
  StraightLineTracker straight_tracker_{};
  double lap_time_seconds_{0.0};
  double lap_distance_meters_{0.0};
};

}  // namespace fsai::sim

