#pragma once

#include "common/include/common/types.h"
#include <vector>

namespace fsai {
namespace vision {

// Internal structure to hold track state
struct TrackedCone {
    int id;
    types::BoxBound bound;
    int lost_frames = 0;
};

class ConeTracker {
public:
    ConeTracker();
    ~ConeTracker();

    /**
     * @brief Updates the tracker with new detections from the current frame.
     * @param detections Raw bounding boxes from YOLOv8
     * @return A vector of BoxBounds with assigned track_ids
     */
    std::vector<types::BoxBound> update(const std::vector<types::BoxBound>& detections);

private:
    // Hyperparameters
    const float MAX_DIST_THRESHOLD = 50.0f; 
    const int MAX_LOST_FRAMES = 5; 

    std::vector<TrackedCone> tracks_;
    int next_id_ = 0;

    // Helper to calculate Euclidean distance between two boxes
    float calculateDistance(const types::BoxBound& a, const types::BoxBound& b);
};

} // namespace vision
} // namespace fsai