#pragma once

#include "sim/mission/MissionDefinition.hpp"

namespace fsai::world::tracks {

class SkidpadTrackStrategy {
public:
    fsai::sim::TrackData Apply(const fsai::sim::TrackData& track) const;
};

}  // namespace fsai::world::tracks

