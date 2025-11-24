#include <cmath>
#include <iostream>

#include "MissionRuntimeState.hpp"

namespace {

bool AlmostEqual(double a, double b, double epsilon = 1e-6) {
  return std::fabs(a - b) <= epsilon;
}

bool TestAccelerationMission() {
  fsai::sim::MissionDefinition def;
  def.descriptor.type = fsai::sim::MissionType::kAcceleration;
  def.targetLaps = 1;

  fsai::sim::MissionRuntimeState state(def);
  state.Update(1.0);
  if (state.run_status() != fsai::sim::MissionRunStatus::kRunning) {
    std::cerr << "Acceleration mission should start running" << std::endl;
    return false;
  }

  state.SetStraightLineProgress(15.0);
  state.SetStraightLineProgress(10.0);
  if (!AlmostEqual(state.straight_line_progress_m(), 15.0)) {
    std::cerr << "Straight line progress should not decrease" << std::endl;
    return false;
  }

  state.RegisterLap(4.5, 75.0);
  if (state.run_status() != fsai::sim::MissionRunStatus::kCompleted) {
    std::cerr << "Acceleration mission should complete after one lap" << std::endl;
    return false;
  }
  double frozen_time = state.mission_time_seconds();
  state.Update(1.0);
  if (!AlmostEqual(state.mission_time_seconds(), frozen_time)) {
    std::cerr << "Mission time should freeze after completion" << std::endl;
    return false;
  }
  state.MarkStopCommanded();
  if (!state.stop_commanded()) {
    std::cerr << "Stop command should be recorded" << std::endl;
    return false;
  }
  return true;
}

bool TestSkidpadMission() {
  fsai::sim::MissionDefinition def;
  def.descriptor.type = fsai::sim::MissionType::kSkidpad;
  def.targetLaps = 4;

  fsai::sim::MissionRuntimeState state(def);
  const auto& segments = state.segments();
  if (segments.size() != 3) {
    std::cerr << "Skidpad should have three segments" << std::endl;
    return false;
  }
  if (segments[0].spec.type != fsai::sim::MissionSegmentType::kWarmup ||
      segments[1].spec.type != fsai::sim::MissionSegmentType::kTimed ||
      segments[2].spec.type != fsai::sim::MissionSegmentType::kExit) {
    std::cerr << "Skidpad segments are not ordered warmup/timed/exit" << std::endl;
    return false;
  }

  state.RegisterLap(10.0, 100.0);
  if (state.current_segment() == nullptr ||
      state.current_segment()->spec.type != fsai::sim::MissionSegmentType::kTimed) {
    std::cerr << "Skidpad should enter timed segment after warmup" << std::endl;
    return false;
  }

  state.RegisterLap(9.5, 100.0);
  if (state.current_segment() == nullptr ||
      state.current_segment()->spec.type != fsai::sim::MissionSegmentType::kTimed) {
    std::cerr << "Skidpad should remain timed until two laps complete" << std::endl;
    return false;
  }

  state.RegisterLap(9.2, 100.0);
  if (state.current_segment() == nullptr ||
      state.current_segment()->spec.type != fsai::sim::MissionSegmentType::kExit) {
    std::cerr << "Skidpad should enter exit segment" << std::endl;
    return false;
  }

  state.RegisterLap(12.0, 100.0);
  if (state.run_status() != fsai::sim::MissionRunStatus::kCompleted) {
    std::cerr << "Skidpad mission should complete after exit lap" << std::endl;
    return false;
  }
  if (state.completed_laps() != def.targetLaps) {
    std::cerr << "Skidpad lap counting mismatch" << std::endl;
    return false;
  }
  return true;
}

bool TestAutocrossMission() {
  fsai::sim::MissionDefinition def;
  def.descriptor.type = fsai::sim::MissionType::kAutocross;
  def.targetLaps = 1;

  fsai::sim::MissionRuntimeState state(def);
  if (state.segments().size() != 1 ||
      state.segments()[0].spec.type != fsai::sim::MissionSegmentType::kTimed) {
    std::cerr << "Autocross should have a single timed segment" << std::endl;
    return false;
  }
  state.RegisterLap(70.0, 1200.0);
  if (state.run_status() != fsai::sim::MissionRunStatus::kCompleted) {
    std::cerr << "Autocross should complete after a single lap" << std::endl;
    return false;
  }
  return true;
}

bool TestTrackdriveMission() {
  fsai::sim::MissionDefinition def;
  def.descriptor.type = fsai::sim::MissionType::kTrackdrive;
  def.targetLaps = 10;

  fsai::sim::MissionRuntimeState state(def);
  if (state.segments().size() != 1) {
    std::cerr << "Trackdrive should have one segment" << std::endl;
    return false;
  }
  for (int lap = 0; lap < 9; ++lap) {
    state.RegisterLap(80.0 + lap, 1500.0);
    if (state.run_status() != fsai::sim::MissionRunStatus::kRunning) {
      std::cerr << "Trackdrive should still be running before lap " << (lap + 1)
                << std::endl;
      return false;
    }
  }
  state.RegisterLap(89.0, 1500.0);
  if (state.run_status() != fsai::sim::MissionRunStatus::kCompleted) {
    std::cerr << "Trackdrive should complete after ten laps" << std::endl;
    return false;
  }
  return true;
}

}  // namespace

int main() {
  bool ok = true;
  ok &= TestAccelerationMission();
  ok &= TestSkidpadMission();
  ok &= TestAutocrossMission();
  ok &= TestTrackdriveMission();
  return ok ? 0 : 1;
}

