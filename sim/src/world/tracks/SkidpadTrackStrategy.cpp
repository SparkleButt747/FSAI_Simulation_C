#include "sim/src/world/tracks/SkidpadTrackStrategy.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::world::tracks {

fsai::sim::TrackData SkidpadTrackStrategy::Apply(const fsai::sim::TrackData& track) const {
    fsai::sim::TrackData adjusted = track;

    const double leftCircleCenterX = -9.25;
    const double rightCircleCenterX = 9.25;
    const double circleCenterZ = 0.0;
    const double circleRadius = 9.0;
    const double corridorMargin = 2.5;

    auto moveCorridorConesToStart = [&](std::vector<Transform>& cones) {
        std::vector<Transform> kept;
        kept.reserve(cones.size());
        for (const auto& cone : cones) {
            const double distLeft = std::hypot(cone.position.x - leftCircleCenterX,
                                               cone.position.z - circleCenterZ);
            const double distRight = std::hypot(cone.position.x - rightCircleCenterX,
                                                cone.position.z - circleCenterZ);
            const bool isCorridorCone = (distLeft > circleRadius + corridorMargin) &&
                                        (distRight > circleRadius + corridorMargin);
            if (isCorridorCone) {
                adjusted.startCones.push_back(cone);
            } else {
                kept.push_back(cone);
            }
        }
        cones.swap(kept);
    };

    moveCorridorConesToStart(adjusted.leftCones);
    moveCorridorConesToStart(adjusted.rightCones);

    std::sort(adjusted.startCones.begin(), adjusted.startCones.end(),
              [](const Transform& a, const Transform& b) {
                  return a.position.z < b.position.z;
              });

    auto flipZ = [](std::vector<Transform>& transforms) {
        for (auto& transform : transforms) {
            transform.position.z = -transform.position.z;
        }
    };

    flipZ(adjusted.startCones);
    flipZ(adjusted.leftCones);
    flipZ(adjusted.rightCones);
    flipZ(adjusted.checkpoints);

    std::swap(adjusted.leftCones, adjusted.rightCones);

    return adjusted;
}

}  // namespace fsai::world::tracks

