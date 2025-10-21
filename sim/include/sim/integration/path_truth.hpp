#pragma once

#include <vector>

#include "Vector.h"
#include "TrackGenerator.hpp"
#include "VehicleState.hpp"

namespace fsai {
namespace integration {

struct PathTruth {
    TrackResult track;  // Raw track output (cones + checkpoints)
    std::vector<Vector3> centerline;  // Extracted checkpoint positions
    std::vector<float> centerline_arc_length;  // cumulative arc length per centerline sample
};

struct PathMeta {
    const std::vector<Vector3>* centerline{nullptr};
    const std::vector<float>* arc_length{nullptr};
    float sample_spacing_m{0.75f};
    float horizon_m{80.0f};
    Vector3 vehicle_position{0.0f, 0.0f, 0.0f};
    float vehicle_yaw{0.0f};
};

PathTruth GeneratePathTruth(const PathConfig& config);

}  // namespace integration
}  // namespace fsai

