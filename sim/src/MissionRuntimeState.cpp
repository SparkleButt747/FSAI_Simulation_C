#include "sim/MissionRuntimeState.hpp"

#include <algorithm>

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

  if (mission_.descriptor.type == MissionType::kSkidpad) {
    skidpad_state = SkidpadState::Approach;
  } else {
    skidpad_state = SkidpadState::NotSkidpad;
  }

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

  if (mission_.descriptor.type == MissionType::kAcceleration) {
    printf("\n\nAcceleration mission: checking for braking condition, track length=%.2f, progress=%.2f\n\n", mission_.track_length_m, straight_line_progress_m_);

    if (mission_.track_length_m > 0.0 && straight_line_progress_m_ >= mission_.track_length_m) {
      run_status_ = MissionRunStatus::kBraking;
    }
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

void MissionRuntimeState::MarkCompleted() {
  run_status_ = MissionRunStatus::kCompleted;
}

void MissionRuntimeState::ConfigureSegments() {
  switch (mission_.descriptor.type) {
    case MissionType::kAcceleration: {
      break;
    }
    case MissionType::kSkidpad: {
      // A full skidpad event is 2 laps on the left circle, and 2 on the right.
      // We will add 1 warmup lap for each.
      segments_.push_back(MakeSegment(MissionSegmentType::kWarmup, 1)); // Warmup Right
      segments_.push_back(MakeSegment(MissionSegmentType::kTimed, 2));  // Timed Right
      segments_.push_back(MakeSegment(MissionSegmentType::kWarmup, 1)); // Warmup Left
      segments_.push_back(MakeSegment(MissionSegmentType::kTimed, 2));  // Timed Left
      break;
    }
    case MissionType::kAutocross:
    case MissionType::kTrackdrive: {
      segments_.push_back(MakeSegment(MissionSegmentType::kTimed, mission_.targetLaps));
      break;
    }
  }
}

}  // namespace fsai::sim

