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

size_t FindForwardStartIndex(const std::vector<Vector3>& centerline,
                             const Vector3& vehicle_position,
                             float vehicle_yaw) {
    if (centerline.empty()) {
        return 0;
    }

    const float cos_yaw = std::cos(vehicle_yaw);
    const float sin_yaw = std::sin(vehicle_yaw);

    size_t best_index = 0;
    float best_score = std::numeric_limits<float>::max();

    for (size_t i = 0; i < centerline.size(); ++i) {
        const Vector3& point = centerline[i];
        const float dx = point.x - vehicle_position.x;
        const float dz = point.z - vehicle_position.z;
        const float dist2 = dx * dx + dz * dz;
        const float forward = dx * cos_yaw + dz * sin_yaw;
        float penalty = 0.0f;
        if (forward < -0.5f) {
            penalty = (std::fabs(forward) - 0.5f) * 100.0f;
        }
        const float score = dist2 + penalty;
        if (score < best_score) {
            best_score = score;
            best_index = i;
        }
    }

    return best_index;
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

    const size_t start_index = FindForwardStartIndex(centerline, start, path->vehicle_yaw);
    const size_t n = centerline.size();

    Vector3 prev = start;
    float traveled = 0.0f;
    float accumulated = 0.0f;

    size_t steps = 0;
    size_t idx = start_index;
    while (out.count < maxN && steps < n) {
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

        idx = (idx + 1) % n;
        ++steps;

        if (idx == start_index) {
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

