#include "TrackBuilder.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "logging.hpp"
#include "sim/mission/TrackCsvLoader.hpp"
#include "cone_constants.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "SkidpadTrackStrategy.hpp"

namespace {

using fsai::sim::kLargeConeMassKg;
using fsai::sim::kLargeConeRadiusMeters;
using fsai::sim::kSmallConeMassKg;
using fsai::sim::kSmallConeRadiusMeters;
using fsai::sim::log::LogWarning;

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

bool hasCheckpoints(const fsai::sim::TrackData& track) {
    return !track.checkpoints.empty();
}

void rebuildConePositions(const std::vector<Cone>& cones, std::vector<Vector3>& positions) {
    positions.clear();
    positions.reserve(cones.size());
    for (const auto& cone : cones) {
        positions.push_back(cone.position);
    }
}

}  // namespace

TrackBuilder::TrackBuilder(const TrackBuilderConfig& config) : config_(config) {
    if (config_.vehicleCollisionRadius <= 0.0f) {
        config_.vehicleCollisionRadius = 0.5f - fsai::sim::kSmallConeRadiusMeters;
    }
}

TrackBuildResult TrackBuilder::Build(const fsai::sim::MissionDefinition& mission) const {
    return buildFromMission(mission);
}

TrackBuildResult TrackBuilder::buildFromMission(
    const fsai::sim::MissionDefinition& mission) const {
    fsai::sim::TrackData track = mission.track;

    if (mission.trackSource == fsai::sim::TrackSource::kRandom &&
        track.checkpoints.empty()) {
        track = generateRandomTrack();
    }

    if (mission.descriptor.type == fsai::sim::MissionType::kSkidpad) {
        track = SkidpadTrackStrategy{}.Rewrite(track);
    }

    if (!hasCheckpoints(track)) {
        LogWarning("MissionDefinition did not provide any checkpoints");
        throw std::runtime_error("MissionDefinition did not provide any checkpoints");
    }

    return buildFromTrackData(track, mission.descriptor.type);
}

TrackBuildResult TrackBuilder::buildFromTrackData(const fsai::sim::TrackData& track,
                                                  fsai::sim::MissionType missionType) const {
    TrackBuildResult result{};
    result.track = track;

    for (const auto& cp : track.checkpoints) {
        result.checkpointPositions.push_back(transformToVector3(cp));
    }

    for (const auto& sc : track.startCones) {
        result.startCones.push_back(makeCone(sc, ConeType::Start));
    }
    for (const auto& lc : track.leftCones) {
        result.leftCones.push_back(makeCone(lc, ConeType::Left));
    }
    for (const auto& rc : track.rightCones) {
        result.rightCones.push_back(makeCone(rc, ConeType::Right));
    }

    rebuildConePositions(result.startCones, result.startConePositions);
    rebuildConePositions(result.leftCones, result.leftConePositions);
    rebuildConePositions(result.rightCones, result.rightConePositions);

    if (!result.leftCones.empty() && !result.rightCones.empty()) {
        const std::size_t gateCount = std::min(result.leftCones.size(), result.rightCones.size());
        result.gateSegments.reserve(gateCount);
        for (std::size_t i = 0; i < gateCount; ++i) {
            Vector2 left{result.leftCones[i].position.x, result.leftCones[i].position.z};
            Vector2 right{result.rightCones[i].position.x, result.rightCones[i].position.z};
            result.gateSegments.push_back(
                makeSegment(left, right, config_.vehicleCollisionRadius));
        }
    }

    auto appendBoundarySegments = [&](const std::vector<Cone>& cones) {
        if (cones.size() < 2) return;

        for (std::size_t i = 0; i + 1 < cones.size(); ++i) {
            Vector2 s{cones[i].position.x, cones[i].position.z};
            Vector2 e{cones[i + 1].position.x, cones[i + 1].position.z};
            result.boundarySegments.push_back(
                makeSegment(s, e, config_.vehicleCollisionRadius));
        }

        Vector2 s{cones.back().position.x, cones.back().position.z};
        Vector2 e{cones.front().position.x, cones.front().position.z};
        result.boundarySegments.push_back(makeSegment(s, e, config_.vehicleCollisionRadius));
    };

    if (missionType != fsai::sim::MissionType::kSkidpad) {
        appendBoundarySegments(result.leftCones);
        appendBoundarySegments(result.rightCones);
    }

    return result;
}

fsai::sim::TrackData TrackBuilder::generateRandomTrack() const {
    constexpr int kMaxRandomAttempts = 3;
    constexpr const char* kFallbackRandomTrack = "configs/tracks/rand.csv";

    for (int attempt = 1; attempt <= kMaxRandomAttempts; ++attempt) {
        PathConfig pathConfig = config_.pathConfig;
        PathGenerator pathGen(pathConfig);
        PathResult path = pathGen.generatePath(pathConfig.resolution);
        TrackGenerator trackGen;
        TrackResult track = trackGen.generateTrack(pathConfig, path);
        fsai::sim::TrackData generated = fsai::sim::TrackData::FromTrackResult(track);
        if (hasCheckpoints(generated)) {
            return generated;
        }

        LogWarning("Random track generation produced no checkpoints; retrying (attempt " +
                    std::to_string(attempt) + "/" +
                    std::to_string(kMaxRandomAttempts) + ")");
    }

    LogWarning("Random track generation failed after retries; using fallback CSV track");
    try {
        const std::filesystem::path fallbackPath{kFallbackRandomTrack};
        fsai::sim::TrackResult track = fsai::sim::LoadTrackFromCsv(fallbackPath);
        fsai::sim::TrackData fallback = fsai::sim::TrackData::FromTrackResult(track);
        if (hasCheckpoints(fallback)) {
            return fallback;
        }
        LogWarning("Fallback CSV track contained no checkpoints");
    } catch (const std::exception& e) {
        LogWarning(std::string("Failed to load fallback CSV track: ") + e.what());
    }

    return {};
}
