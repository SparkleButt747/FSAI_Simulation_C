#include "TrackBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "cone_constants.hpp"
#include "SkidpadTrackStrategy.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"

namespace {

using fsai::sim::kLargeConeMassKg;
using fsai::sim::kLargeConeRadiusMeters;
using fsai::sim::kSmallConeMassKg;
using fsai::sim::kSmallConeRadiusMeters;

Vector3 transformToVector3(const Transform& t) {
    return {t.position.x, t.position.y, t.position.z};
}

Cone makeCone(const Transform& t, ConeType type) {
    Cone cone;
    cone.position = transformToVector3(t);
    cone.type = type;
    switch (type) {
        case ConeType::Start:
            cone.radius = kLargeConeRadiusMeters;
            cone.mass = kLargeConeMassKg;
            break;
        case ConeType::Left:
        case ConeType::Right:
            cone.radius = kSmallConeRadiusMeters;
            cone.mass = kSmallConeMassKg;
            break;
    }
    return cone;
}

CollisionSegment makeSegment(const Vector2& start, const Vector2& end, float radius) {
    CollisionSegment segment{};
    segment.start = start;
    segment.end = end;
    segment.radius = radius;
    const float minX = std::min(start.x, end.x) - radius;
    const float maxX = std::max(start.x, end.x) + radius;
    const float minY = std::min(start.y, end.y) - radius;
    const float maxY = std::max(start.y, end.y) + radius;
    segment.boundsMin = Vector2{minX, minY};
    segment.boundsMax = Vector2{maxX, maxY};
    return segment;
}

}  // namespace

TrackBuildResult TrackBuilder::Build(const fsai::sim::MissionDefinition& mission,
                                     const PathConfig& pathConfig,
                                     float vehicleCollisionRadius) const {
    fsai::sim::TrackData trackData = BuildTrackData(mission, pathConfig);

    if (trackData.checkpoints.empty()) {
        throw std::runtime_error("MissionDefinition did not provide any checkpoints");
    }

    const bool isSkidpad = mission.descriptor.type == fsai::sim::MissionType::kSkidpad;
    const fsai::sim::TrackData processed = ApplyStrategies(mission, trackData);

    return DeriveState(processed, vehicleCollisionRadius, isSkidpad);
}

fsai::sim::TrackData TrackBuilder::BuildTrackData(const fsai::sim::MissionDefinition& mission,
                                                  const PathConfig& pathConfig) const {
    if (mission.trackSource != fsai::sim::TrackSource::kRandom ||
        !mission.track.checkpoints.empty()) {
        return mission.track;
    }

    PathConfig config = pathConfig;
    config.calculateResolutionAndLength();
    const int nPoints = config.resolution;
    PathGenerator pathGenerator(config);
    const PathResult path = pathGenerator.generatePath(nPoints);
    TrackGenerator trackGenerator;
    const TrackResult track = trackGenerator.generateTrack(config, path);
    return fsai::sim::TrackData::FromTrackResult(track);
}

fsai::sim::TrackData TrackBuilder::ApplyStrategies(const fsai::sim::MissionDefinition& mission,
                                                   const fsai::sim::TrackData& track) const {
    if (mission.descriptor.type == fsai::sim::MissionType::kSkidpad) {
        fsai::world::tracks::SkidpadTrackStrategy strategy;
        return strategy.Apply(track);
    }

    return track;
}

TrackBuildResult TrackBuilder::DeriveState(const fsai::sim::TrackData& track,
                                           float vehicleCollisionRadius,
                                           bool isSkidpad) const {
    TrackBuildResult state;
    state.track = track;
    state.checkpointPositions.reserve(track.checkpoints.size());
    for (const auto& checkpoint : track.checkpoints) {
        state.checkpointPositions.push_back(transformToVector3(checkpoint));
    }

    state.startCones.reserve(track.startCones.size());
    for (const auto& cone : track.startCones) {
        state.startCones.push_back(makeCone(cone, ConeType::Start));
    }

    state.leftCones.reserve(track.leftCones.size());
    for (const auto& cone : track.leftCones) {
        state.leftCones.push_back(makeCone(cone, ConeType::Left));
    }

    state.rightCones.reserve(track.rightCones.size());
    for (const auto& cone : track.rightCones) {
        state.rightCones.push_back(makeCone(cone, ConeType::Right));
    }

    auto rebuildConePositions = [&](std::vector<Vector3>& positions, const std::vector<Cone>& cones) {
        positions.clear();
        positions.reserve(cones.size());
        for (const auto& cone : cones) {
            positions.push_back(cone.position);
        }
    };

    rebuildConePositions(state.startConePositions, state.startCones);
    rebuildConePositions(state.leftConePositions, state.leftCones);
    rebuildConePositions(state.rightConePositions, state.rightCones);

    if (!state.leftCones.empty() && !state.rightCones.empty()) {
        const std::size_t gateCount = std::min(state.leftCones.size(), state.rightCones.size());
        state.gateSegments.reserve(gateCount);
        for (std::size_t i = 0; i < gateCount; ++i) {
            const Vector2 left{state.leftCones[i].position.x, state.leftCones[i].position.z};
            const Vector2 right{state.rightCones[i].position.x, state.rightCones[i].position.z};
            state.gateSegments.push_back(makeSegment(left, right, vehicleCollisionRadius));
        }
    }

    auto appendBoundarySegments = [&](const std::vector<Cone>& cones) {
        if (cones.size() < 2) return;

        for (std::size_t i = 0; i + 1 < cones.size(); ++i) {
            const Vector2 start{cones[i].position.x, cones[i].position.z};
            const Vector2 end{cones[i + 1].position.x, cones[i + 1].position.z};
            state.boundarySegments.push_back(makeSegment(start, end, vehicleCollisionRadius));
        }

        const Vector2 loopStart{cones.back().position.x, cones.back().position.z};
        const Vector2 loopEnd{cones.front().position.x, cones.front().position.z};
        state.boundarySegments.push_back(makeSegment(loopStart, loopEnd, vehicleCollisionRadius));
    };

    if (!isSkidpad) {
        appendBoundarySegments(state.leftCones);
        appendBoundarySegments(state.rightCones);
    }

    if (!track.checkpoints.empty()) {
        state.lastCheckpoint = transformToVector3(track.checkpoints.back());
    }

    return state;
}

