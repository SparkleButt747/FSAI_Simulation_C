#include "SkidpadTrackStrategy.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kLeftCircleCenterX = -9.25;
constexpr double kRightCircleCenterX = 9.25;
constexpr double kCircleCenterZ = 0.0;
constexpr double kCircleRadius = 9.0;
constexpr double kCorridorMargin = 2.5;

bool IsCorridorCone(const Transform& cone) {
    const double distLeft =
        std::hypot(static_cast<double>(cone.position.x) - kLeftCircleCenterX,
                   static_cast<double>(cone.position.z) - kCircleCenterZ);
    const double distRight =
        std::hypot(static_cast<double>(cone.position.x) - kRightCircleCenterX,
                   static_cast<double>(cone.position.z) - kCircleCenterZ);
    return distLeft > kCircleRadius + kCorridorMargin &&
           distRight > kCircleRadius + kCorridorMargin;
}

void FlipZ(std::vector<Transform>& transforms) {
    for (auto& transform : transforms) {
        transform.position.z = -transform.position.z;
    }
}

}  // namespace

fsai::sim::TrackData SkidpadTrackStrategy::Rewrite(
    const fsai::sim::TrackData& track) const {
    fsai::sim::TrackData rewritten = track;

    auto moveCorridorCones = [&](std::vector<Transform>& cones) {
        std::vector<Transform> keep;
        keep.reserve(cones.size());
        for (const auto& cone : cones) {
            if (IsCorridorCone(cone)) {
                rewritten.startCones.push_back(cone);
            } else {
                keep.push_back(cone);
            }
        }
        cones.swap(keep);
    };

    moveCorridorCones(rewritten.leftCones);
    moveCorridorCones(rewritten.rightCones);

    std::sort(rewritten.startCones.begin(), rewritten.startCones.end(),
              [](const Transform& a, const Transform& b) {
                  return a.position.z < b.position.z;
              });

    FlipZ(rewritten.startCones);
    FlipZ(rewritten.leftCones);
    FlipZ(rewritten.rightCones);
    FlipZ(rewritten.checkpoints);

    std::swap(rewritten.leftCones, rewritten.rightCones);

    return rewritten;
}
