#include "sim/integration/adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsai {
namespace integration {
namespace {

struct SidePoint {
    float forward;
    Vector3 world;
};

float DistanceXZ(const Vector3& a, const Vector3& b) {
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dz * dz);
}

AdapterOutput BuildFromPath(const PathMeta* path,
                            Vector3* checkpointPositions,
                            int maxN,
                            float sample_m,
                            float horizon_m) {
    AdapterOutput out{};
    out.status |= AD_STATUS_FALLBACK_PATH;
    if (!path || !path->centerline || path->centerline->empty()) {
        return out;
    }

    const std::vector<Vector3>& centerline = *path->centerline;

    float spacing = sample_m > 0.0f ? sample_m : path->sample_spacing_m;
    float horizon = horizon_m > 0.0f ? horizon_m : path->horizon_m;

    Vector3 start = path->vehicle_position;

    // Find closest index ahead of the vehicle.
    size_t closest = 0;
    float best_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < centerline.size(); ++i) {
        const float dist = DistanceXZ(centerline[i], start);
        if (dist < best_dist) {
            best_dist = dist;
            closest = i;
        }
    }

    Vector3 prev = start;
    float traveled = 0.0f;
    float accumulated = 0.0f;

    for (size_t idx = closest; idx < centerline.size() && out.count < maxN; ++idx) {
        const Vector3& point = centerline[idx];
        const float step = DistanceXZ(point, prev);
        traveled += step;
        accumulated += step;
        if (accumulated >= spacing || out.count == 0) {
            checkpointPositions[out.count++] = point;
            accumulated = 0.0f;
        }
        prev = point;
        if (traveled >= horizon) {
            break;
        }
    }

    return out;
}

}  // namespace

AdapterOutput Adapter_BuildCheckpoints(const FsaiDetections* dets_or_null,
                                       const PathMeta* path_or_null,
                                       Vector3* checkpointPositions,
                                       int maxN,
                                       float sample_m,
                                       float horizon_m) {
    AdapterOutput output{};
    if (maxN <= 0 || !checkpointPositions) {
        return output;
    }

    float spacing = sample_m > 0.0f ? sample_m : (path_or_null ? path_or_null->sample_spacing_m : 1.0f);
    float horizon = horizon_m > 0.0f ? horizon_m : (path_or_null ? path_or_null->horizon_m : 30.0f);

    if (dets_or_null && dets_or_null->n > 0 && path_or_null) {
        std::vector<SidePoint> left;
        std::vector<SidePoint> right;
        left.reserve(dets_or_null->n);
        right.reserve(dets_or_null->n);

        const float yaw = path_or_null->vehicle_yaw;
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);

        for (int i = 0; i < dets_or_null->n; ++i) {
            const FsaiConeDet& det = dets_or_null->dets[i];
            Vector3 world{det.p_B[0], det.p_B[1], det.p_B[2]};
            const float rel_x = world.x - path_or_null->vehicle_position.x;
            const float rel_z = world.z - path_or_null->vehicle_position.z;
            float lateral = -rel_x * sin_yaw + rel_z * cos_yaw;
            float forward = rel_x * cos_yaw + rel_z * sin_yaw;
            FsaiConeSide side = det.side;
            if (side == FSAI_CONE_UNKNOWN) {
                side = lateral >= 0.0f ? FSAI_CONE_LEFT : FSAI_CONE_RIGHT;
            }
            if (forward <= 0.0f) {
                continue;
            }

            SidePoint pt{forward, world};
            if (side == FSAI_CONE_LEFT) {
                left.push_back(pt);
            } else if (side == FSAI_CONE_RIGHT) {
                right.push_back(pt);
            }
        }

        if (left.empty()) {
            output.status |= AD_STATUS_NO_LEFT;
        }
        if (right.empty()) {
            output.status |= AD_STATUS_NO_RIGHT;
        }

        std::sort(left.begin(), left.end(), [](const SidePoint& a, const SidePoint& b) { return a.forward < b.forward; });
        std::sort(right.begin(), right.end(), [](const SidePoint& a, const SidePoint& b) { return a.forward < b.forward; });

        std::vector<SidePoint> midpoints;
        size_t li = 0, ri = 0;
        const float pair_threshold = 2.5f;
        while (li < left.size() && ri < right.size()) {
            const SidePoint& L = left[li];
            const SidePoint& R = right[ri];
            if (std::fabs(L.forward - R.forward) > pair_threshold) {
                if (L.forward < R.forward) {
                    ++li;
                } else {
                    ++ri;
                }
                continue;
            }
            SidePoint mid{};
            mid.forward = 0.5f * (L.forward + R.forward);
            mid.world.x = 0.5f * (L.world.x + R.world.x);
            mid.world.y = 0.0f;
            mid.world.z = 0.5f * (L.world.z + R.world.z);
            midpoints.push_back(mid);
            ++li;
            ++ri;
        }

        if (!midpoints.empty()) {
            Vector3 origin = path_or_null->vehicle_position;
            float traveled = 0.0f;
            float accumulated = 0.0f;
            Vector3 prev = origin;

            for (const SidePoint& mid : midpoints) {
                const float step = DistanceXZ(mid.world, prev);
                traveled += step;
                accumulated += step;
                if (accumulated >= spacing || output.count == 0) {
                    checkpointPositions[output.count++] = mid.world;
                    accumulated = 0.0f;
                    if (output.count >= maxN) {
                        break;
                    }
                }
                prev = mid.world;
                if (traveled >= horizon) {
                    break;
                }
            }
        }
    }

    if (output.count == 0) {
        AdapterOutput fallback = BuildFromPath(path_or_null, checkpointPositions, maxN, spacing, horizon);
        output.count = fallback.count;
        output.status |= fallback.status;
    }

    return output;
}

}  // namespace integration
}  // namespace fsai

