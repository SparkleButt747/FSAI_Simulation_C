#pragma once

#include "common/types.h"
#include <vector>
#include <Eigen/Core> // Needed for Eigen::Vector2d

namespace fsai {
namespace vision {

// Internal representation of a cone in the map
struct MapCone {
    float x;         // Global X
    float y;         // Global Y (mapped from input Z)
    int side;        // Cone Color
    float conf;      // Accumulated Confidence / Weight
    int seen_count;  // Number of times updated
};

class SimpleMap {
public:
    SimpleMap();
    ~SimpleMap();

    /**
     * @brief Updates the map with new global detections from the current frame.
     * @param new_detections Vector of cones in global coords (x, z)
     * @param car_pos Current car position (x, y) for proximity weighting
     */
    void update(const std::vector<FsaiConeDet>& new_detections, const Eigen::Vector2d& car_pos);
    void clearMap();
    // Direct access to the map for the main loop to read from
    std::vector<MapCone> cones;

private:
    // --- Tunable Parameters ---
    const float MERGE_RADIUS = 2.0f;   // Meters. Cones closer than this are merged.
    const float MAX_CONF_CAP = 10.0f;  // Maximum weight a cone can accumulate.
    const float REF_DIST = 10.0f;      // Distance at which weight is halved (for proximity calculation)
};

} // namespace vision
} // namespace fsai