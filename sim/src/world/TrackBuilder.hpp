#pragma once

#include "mission/MissionDefinition.hpp"
#include "track/PathConfig.hpp"
#include "TrackState.hpp"

class TrackBuilder {
public:
    TrackBuildResult Build(const fsai::sim::MissionDefinition& mission,
                           const PathConfig& pathConfig,
                           float vehicleCollisionRadius) const;

private:
    fsai::sim::TrackData BuildTrackData(const fsai::sim::MissionDefinition& mission,
                                        const PathConfig& pathConfig) const;
    fsai::sim::TrackData ApplyStrategies(const fsai::sim::MissionDefinition& mission,
                                         const fsai::sim::TrackData& track) const;
    TrackBuildResult DeriveState(const fsai::sim::TrackData& track,
                                 float vehicleCollisionRadius,
                                 bool isSkidpad) const;
};

