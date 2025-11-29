#pragma once

#include <vector>

#include "sim/mission/MissionDefinition.hpp"
#include "sim/track/PathConfig.hpp"
#include "sim/world/TrackTypes.hpp"

struct TrackBuilderConfig {
    PathConfig pathConfig{};
    float vehicleCollisionRadius{0.5f};
};

struct TrackBuildResult {
    fsai::sim::TrackData track{};
    std::vector<Vector3> checkpointPositions;
    std::vector<Cone> startCones;
    std::vector<Cone> leftCones;
    std::vector<Cone> rightCones;
    std::vector<Cone> orangeCones;
    std::vector<Vector3> startConePositions;
    std::vector<Vector3> leftConePositions;
    std::vector<Vector3> rightConePositions;
    std::vector<Vector3> orangeConePositions;
    std::vector<CollisionSegment> gateSegments;
    std::vector<CollisionSegment> boundarySegments;
};

class TrackBuilder {
public:
    explicit TrackBuilder(const TrackBuilderConfig& config);

    TrackBuildResult Build(const fsai::sim::MissionDefinition& mission) const;

private:
    TrackBuilderConfig config_{};

    TrackBuildResult buildFromTrackData(const fsai::sim::TrackData& track,
                                        fsai::sim::MissionType missionType) const;
    TrackBuildResult buildFromMission(const fsai::sim::MissionDefinition& mission) const;
    fsai::sim::TrackData generateRandomTrack() const;
};
