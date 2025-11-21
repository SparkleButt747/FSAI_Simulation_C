#include "Telemetry.hpp"
#include "fsai_clock.h"
#include <cstdio>

namespace {

const char* SegmentTypeToString(fsai::sim::MissionSegmentType type) {
    switch (type) {
        case fsai::sim::MissionSegmentType::kWarmup:
            return "warmup";
        case fsai::sim::MissionSegmentType::kTimed:
            return "timed";
        case fsai::sim::MissionSegmentType::kExit:
            return "exit";
    }
    return "unknown";
}

const char* RunStatusToString(fsai::sim::MissionRunStatus status) {
    switch (status) {
        case fsai::sim::MissionRunStatus::kRunning:
            return "running";
        case fsai::sim::MissionRunStatus::kBraking:
            return "braking";
        case fsai::sim::MissionRunStatus::kCompleted:
            return "completed";
    }
    return "unknown";
}

}  // namespace

void Telemetry_Update(const VehicleState& carState, const Transform& carTransform,
                      uint64_t simTimeNs, double lapTimeSeconds,
                      double totalDistance, int lapCount,
                      const fsai::sim::MissionRuntimeState& missionState) {
    const double simTimeSeconds = fsai_clock_to_seconds(simTimeNs);
    const std::size_t targetLaps = missionState.target_laps();
    const auto* segment = missionState.current_segment();
    const char* segmentName = segment ? SegmentTypeToString(segment->spec.type) : "complete";
    const std::size_t segmentTarget = segment ? segment->spec.laps : 0;
    const std::size_t segmentCompleted = segment ? segment->completed_laps : segmentTarget;

    std::printf("Run Time: %.3f s | Mission Time: %.3f s\n",
                simTimeSeconds, missionState.mission_time_seconds());
    std::printf("Mission Status: %s | Segment: %s (%zu/%zu laps)\n",
                RunStatusToString(missionState.run_status()),
                segmentName, segmentCompleted, segmentTarget);
    std::printf("Lap Time: %.3f s | Lap Distance: %.3f m | Laps: %d/%zu\n",
                lapTimeSeconds, totalDistance, lapCount, targetLaps);

    const double straightProgress = missionState.straight_line_progress_m();
    if (straightProgress > 0.0) {
        std::printf("Straight Progress: %.2f m\n", straightProgress);
    }
}
