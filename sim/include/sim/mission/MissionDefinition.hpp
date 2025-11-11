#pragma once

#include <cstddef>
#include <vector>
#include <utility>

#include "Transform.h"
#include "sim/mission_descriptor.hpp"
#include "sim/track/TrackGenerator.hpp"

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
  std::vector<std::pair<Transform, Transform>> gates;

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
  data.gates = track.gates;
  return data;
}

}  // namespace fsai::sim

