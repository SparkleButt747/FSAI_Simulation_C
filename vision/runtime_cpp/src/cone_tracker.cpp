#include "cone_tracker.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace fsai {
namespace vision {

ConeTracker::ConeTracker() = default;
ConeTracker::~ConeTracker() = default;

std::vector<types::BoxBound> ConeTracker::update(const std::vector<types::BoxBound>& detections) {
    std::vector<types::BoxBound> result_bounds;
    std::vector<bool> matched_detection(detections.size(), false);

    // 1. Try to match existing tracks to new detections
    for (auto& track : tracks_) {
        track.lost_frames++; // Assume lost until found
        
        float best_dist = MAX_DIST_THRESHOLD;
        int best_det_idx = -1;

        // Find closest detection to this track
        for (size_t i = 0; i < detections.size(); ++i) {
            if (matched_detection[i]) continue; // Already matched to another track

            float dist = calculateDistance(track.bound, detections[i]);
            if (dist < best_dist) {
                best_dist = dist;
                best_det_idx = (int)i;
            }
        }

        // If we found a match
        if (best_det_idx != -1) {
            matched_detection[best_det_idx] = true;
            track.lost_frames = 0;
            track.bound = detections[best_det_idx]; // Update position
            track.bound.track_id = track.id;       // Keep the existing ID
            result_bounds.push_back(track.bound);
        }
    }

    // 2. Remove dead tracks (lost for too many frames)
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const TrackedCone& t) { 
            return t.lost_frames > MAX_LOST_FRAMES; 
        }),
        tracks_.end());

    // 3. Create new tracks for unmatched detections
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!matched_detection[i]) {
            TrackedCone new_track;
            new_track.id = next_id_++;
            new_track.bound = detections[i];
            new_track.bound.track_id = new_track.id;
            
            tracks_.push_back(new_track);
            result_bounds.push_back(new_track.bound);
        }
    }

    return result_bounds;
}

float ConeTracker::calculateDistance(const types::BoxBound& a, const types::BoxBound& b) {
    // Calculate center points
    float cx_a = a.x + a.w / 2.0f;
    float cy_a = a.y + a.h / 2.0f;
    float cx_b = b.x + b.w / 2.0f;
    float cy_b = b.y + b.h / 2.0f;

    // Euclidean distance
    return std::sqrt(std::pow(cx_a - cx_b, 2) + std::pow(cy_a - cy_b, 2));
}

} // namespace vision
} // namespace fsai