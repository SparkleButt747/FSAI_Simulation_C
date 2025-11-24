#pragma once

#include <cstddef>
#include <vector>

#include "Transform.h"
#include "mission_descriptor.hpp"
#include "track/TrackGenerator.hpp"

namespace fsai::sim {

enum class TrackSource {
  kRandom,
  kCsv,
};

struct TrackData {
  std::vector<Transform> startCones;
  std::vector<Transform> leftCones;
  std::vector<Transform> rightCones;
  std::vector<Transform> checkpoints;

  static TrackData FromTrackResult(const TrackResult& track);
};

struct MissionDefinition {
  MissionDescriptor descriptor{};
  TrackData track{};
  std::size_t targetLaps{1};
  bool allowRegeneration{true};
  TrackSource trackSource{TrackSource::kRandom};
};

inline TrackData TrackData::FromTrackResult(const TrackResult& track) {
  TrackData data;
  data.startCones = track.startCones;
  data.leftCones = track.leftCones;
  data.rightCones = track.rightCones;
  data.checkpoints = track.checkpoints;
  return data;
}

}  // namespace fsai::sim

