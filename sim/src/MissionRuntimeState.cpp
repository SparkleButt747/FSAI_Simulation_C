#include "sim/MissionRuntimeState.hpp"

#include <algorithm>
#include <limits>

namespace fsai::sim {

namespace {

MissionSegmentRuntime MakeSegment(MissionSegmentType type, std::size_t laps) {
  MissionSegmentRuntime runtime;
  runtime.spec.type = type;
  runtime.spec.laps = laps;
  runtime.completed_laps = 0;
  runtime.elapsed_time_s = 0.0;
  return runtime;
}

}  // namespace

MissionRuntimeState::MissionRuntimeState(const MissionDefinition& definition) {
  Reset(definition);
}

void MissionRuntimeState::Reset(const MissionDefinition& definition) {
  mission_ = definition;
  segments_.clear();
  current_segment_index_ = 0;
  total_completed_laps_ = 0;
  mission_time_seconds_ = 0.0;
  straight_line_progress_m_ = 0.0;
  last_lap_time_s_ = 0.0;
  last_lap_distance_m_ = 0.0;
  run_status_ = MissionRunStatus::kRunning;
  stop_commanded_ = false;

  ConfigureSegments();

  total_target_laps_ = 0;
  for (const auto& segment : segments_) {
    total_target_laps_ += segment.spec.laps;
  }
}

void MissionRuntimeState::Update(double dt_seconds) {
  if (run_status_ == MissionRunStatus::kCompleted) {
    return;
  }
  mission_time_seconds_ += dt_seconds;
  if (auto* segment = ActiveSegment(); segment != nullptr) {
    segment->elapsed_time_s += dt_seconds;
  }
}

void MissionRuntimeState::RegisterLap(double lap_time_s, double lap_distance_m) {
  if (run_status_ == MissionRunStatus::kCompleted) {
    return;
  }
  last_lap_time_s_ = lap_time_s;
  last_lap_distance_m_ = lap_distance_m;
  ++total_completed_laps_;

  if (auto* segment = ActiveSegment(); segment != nullptr) {
    ++segment->completed_laps;
    if (segment->completed_laps >= segment->spec.laps) {
      ++current_segment_index_;
      if (current_segment_index_ >= segments_.size()) {
        run_status_ = MissionRunStatus::kCompleted;
      }
    }
  } else {
    run_status_ = MissionRunStatus::kCompleted;
  }
}

void MissionRuntimeState::SetStraightLineProgress(double progress_meters) {
  straight_line_progress_m_ = std::max(straight_line_progress_m_, progress_meters);
}

const MissionSegmentRuntime* MissionRuntimeState::current_segment() const {
  if (current_segment_index_ >= segments_.size()) {
    return nullptr;
  }
  return &segments_[current_segment_index_];
}

MissionSegmentRuntime* MissionRuntimeState::ActiveSegment() {
  if (current_segment_index_ >= segments_.size()) {
    return nullptr;
  }
  return &segments_[current_segment_index_];
}

void MissionRuntimeState::MarkStopCommanded() {
  stop_commanded_ = true;
}

void MissionRuntimeState::ConfigureSegments() {
  switch (mission_.descriptor.type) {
    case MissionType::kAcceleration: {
      segments_.push_back(MakeSegment(MissionSegmentType::kTimed, mission_.targetLaps));
      break;
    }
    case MissionType::kSkidpad: {
      const std::size_t warmup_laps = mission_.targetLaps >= 3 ? 1 : 0;
      const std::size_t exit_laps = mission_.targetLaps >= 3 ? 1 : 0;
      const std::size_t timed_laps = mission_.targetLaps > warmup_laps + exit_laps
                                         ? mission_.targetLaps - warmup_laps - exit_laps
                                         : 0;
      if (warmup_laps > 0) {
        segments_.push_back(MakeSegment(MissionSegmentType::kWarmup, warmup_laps));
      }
      if (timed_laps > 0) {
        segments_.push_back(MakeSegment(MissionSegmentType::kTimed, timed_laps));
      }
      if (exit_laps > 0) {
        segments_.push_back(MakeSegment(MissionSegmentType::kExit, exit_laps));
      }
      break;
    }
    case MissionType::kAutocross:
    case MissionType::kTrackdrive: {
      segments_.push_back(MakeSegment(MissionSegmentType::kTimed, mission_.targetLaps));
      break;
    }
    case MissionType::kSandbox: {
      const std::size_t laps = mission_.targetLaps > 0
                                   ? mission_.targetLaps
                                   : std::numeric_limits<std::size_t>::max();
      segments_.push_back(MakeSegment(MissionSegmentType::kTimed, laps));
      break;
    }
  }
}

}  // namespace fsai::sim

