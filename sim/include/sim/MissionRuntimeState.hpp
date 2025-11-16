#pragma once

#include <cstddef>
#include <vector>

#include "sim/mission/MissionDefinition.hpp"

namespace fsai::sim {

enum class MissionSegmentType {
  kWarmup,
  kTimed,
  kExit,
};

enum class MissionRunStatus {
  kRunning,
  kCompleted,
};

struct MissionSegmentSpec {
  MissionSegmentType type{MissionSegmentType::kTimed};
  std::size_t laps{0};
};

struct MissionSegmentRuntime {
  MissionSegmentSpec spec{};
  std::size_t completed_laps{0};
  double elapsed_time_s{0.0};
};

class MissionRuntimeState {
 public:
  MissionRuntimeState() = default;
  explicit MissionRuntimeState(const MissionDefinition& definition);

  void Reset(const MissionDefinition& definition);
  void Update(double dt_seconds);
  void RegisterLap(double lap_time_s, double lap_distance_m);
  void SetStraightLineProgress(double progress_meters);

  MissionRunStatus run_status() const { return run_status_; }
  std::size_t completed_laps() const { return total_completed_laps_; }
  std::size_t target_laps() const { return total_target_laps_; }
  double mission_time_seconds() const { return mission_time_seconds_; }
  double straight_line_progress_m() const { return straight_line_progress_m_; }
  double last_lap_time_seconds() const { return last_lap_time_s_; }
  double last_lap_distance_meters() const { return last_lap_distance_m_; }
  const MissionDefinition& mission() const { return mission_; }
  const MissionSegmentRuntime* current_segment() const;
  const std::vector<MissionSegmentRuntime>& segments() const { return segments_; }
  bool mission_complete() const { return run_status_ == MissionRunStatus::kCompleted; }
  bool stop_commanded() const { return stop_commanded_; }
  void MarkStopCommanded();

 private:
  void ConfigureSegments();
  MissionSegmentRuntime* ActiveSegment();

  MissionDefinition mission_{};
  std::vector<MissionSegmentRuntime> segments_{};
  std::size_t current_segment_index_{0};
  std::size_t total_completed_laps_{0};
  std::size_t total_target_laps_{0};
  double mission_time_seconds_{0.0};
  double straight_line_progress_m_{0.0};
  double last_lap_time_s_{0.0};
  double last_lap_distance_m_{0.0};
  MissionRunStatus run_status_{MissionRunStatus::kRunning};
  bool stop_commanded_{false};
};

}  // namespace fsai::sim

