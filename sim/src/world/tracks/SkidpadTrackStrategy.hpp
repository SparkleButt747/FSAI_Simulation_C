#pragma once

#include "sim/mission/MissionDefinition.hpp"

class SkidpadTrackStrategy {
public:
    fsai::sim::TrackData Rewrite(const fsai::sim::TrackData& track) const;
};
