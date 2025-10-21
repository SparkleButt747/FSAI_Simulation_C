#include "sim/integration/path_truth.hpp"

#include <cmath>

namespace fsai {
namespace integration {
namespace {

static std::vector<Vector3> ExtractCenterline(const TrackResult& track) {
    std::vector<Vector3> result;
    result.reserve(track.checkpoints.size());
    for (const Transform& t : track.checkpoints) {
        result.push_back(t.position);
    }
    return result;
}

static std::vector<float> ComputeArcLength(const std::vector<Vector3>& pts) {
    std::vector<float> arc;
    arc.reserve(pts.size());
    float accum = 0.0f;
    arc.push_back(0.0f);
    for (size_t i = 1; i < pts.size(); ++i) {
        const Vector3& a = pts[i - 1];
        const Vector3& b = pts[i];
        const float dx = b.x - a.x;
        const float dz = b.z - a.z;
        const float dist = std::sqrt(dx * dx + dz * dz);
        accum += dist;
        arc.push_back(accum);
    }
    return arc;
}

}  // namespace

PathTruth GeneratePathTruth(const PathConfig& config) {
    PathGenerator generator(config);
    const int nPoints = config.resolution;
    PathResult path = generator.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(config, path);

    PathTruth truth;
    truth.track = std::move(track);
    truth.centerline = ExtractCenterline(truth.track);
    truth.centerline_arc_length = ComputeArcLength(truth.centerline);
    return truth;
}

}  // namespace integration
}  // namespace fsai

