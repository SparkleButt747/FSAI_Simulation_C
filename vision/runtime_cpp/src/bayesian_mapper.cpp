#include "bayesian_mapper.hpp"
#include <iostream>
#include <algorithm> // for std::max

namespace fsai {
namespace vision {

BayesianMapper::BayesianMapper() = default;

void BayesianMapper::updateMap(const std::vector<FsaiConeDet>& global_detections, const CarState& car) {
    
    for (const auto& det : global_detections) {

        // 1. Map Inputs to Internal Grid
        // Input: det.x (Global East), det.z (Global North)
        // Map:   x, y
        double meas_x = static_cast<double>(det.x);
        double meas_y = static_cast<double>(det.y);

        // 2. Calculate Variance Heuristic (Since upstream cov is empty)

        // A. Get Distance squared from Car to this Detection
        // We need this because error scales with distance from the sensor
        double dist_sq = std::pow(meas_x - car.x, 2) + std::pow(meas_y - car.y, 2);

        // B. Calculate Distance Noise component
        // BASE_SENSOR_NOISE (e.g. 0.05m) is the error at 0 meters.
        // DIST_NOISE_FACTOR (e.g. 0.1) means error grows by 10cm for every meter away.
        double dist_noise = BASE_SENSOR_NOISE + (dist_sq * (DIST_NOISE_FACTOR * DIST_NOISE_FACTOR));
        
        // C. Apply Confidence Penalty
        // We square the confidence to punish low-confidence detections heavily.
        // Clamp confidence to 0.1 to prevent division by zero.
        double safe_conf = std::max(0.1f, det.conf);
        double meas_variance = dist_noise / (safe_conf * safe_conf);

        // 3. Data Association (Nearest Neighbor)
        int best_idx = -1;
        double min_dist_sq = ASSOCIATION_THRESH * ASSOCIATION_THRESH;

        for (size_t i = 0; i < map_cones_.size(); ++i) {
            // Calculate distance between New Measurement and Existing Map Landmark
            double dx = map_cones_[i].x - meas_x;
            double dy = map_cones_[i].y - meas_y;
            double d_sq = dx*dx + dy*dy;

            if (d_sq < min_dist_sq) {
                min_dist_sq = d_sq;
                best_idx = (int)i;
            }
        }

        // 4. Execute Recursive Bayesian Update
        if (best_idx != -1) {
            // --- MATCH: Update existing cone ---
            bayesianUpdate(map_cones_[best_idx], meas_x, meas_y, meas_variance);
            
            // Accumulate confidence into the Trust Score
            map_cones_[best_idx].trust_score += det.conf;
            
            // Cap the trust score (e.g., max 15.0) so it doesn't grow infinitely
            if(map_cones_[best_idx].trust_score > 15.0) map_cones_[best_idx].trust_score = 15.0;

            // (Optional) Update color?
            // Usually we trust the color of the closest/highest confidence detection
            // or implement a voting system. Keeping it simple: don't overwrite color.

        } else {
            // --- NO MATCH: Add new cone ---
            MapCone new_cone;
            new_cone.id = global_id_counter_++;
            new_cone.x = meas_x;
            new_cone.y = meas_y;
            new_cone.variance = meas_variance; 
            new_cone.side = det.side;
            new_cone.trust_score = det.conf; // Initialize trust with current confidence
            new_cone.is_validated = false;
            
            map_cones_.push_back(new_cone);
        }
    }
}

void BayesianMapper::bayesianUpdate(MapCone& cone, double meas_x, double meas_y, double meas_var) {
    // 1. Calculate Kalman Gain (K)
    // K approaches 1.0 if our map is unsure (high cone.variance) but measurement is precise (low meas_var)
    // K approaches 0.0 if our map is solid (low cone.variance) but measurement is noisy
    double K = cone.variance / (cone.variance + meas_var);

    // 2. Update Position (Mean)
    // New = Old + K * (Measurement - Old)
    cone.x = cone.x + K * (meas_x - cone.x);
    cone.y = cone.y + K * (meas_y - cone.y);

    // 3. Update Uncertainty (Variance)
    // Variance shrinks every time we get a new measurement
    cone.variance = (1.0 - K) * cone.variance;
}

std::vector<MapCone> BayesianMapper::getValidatedMap() {
    std::vector<MapCone> valid_cones;
    // Pre-allocate to avoid reallocations (optimization)
    valid_cones.reserve(map_cones_.size());

    for(auto& cone : map_cones_) {
        // Filter: Only return cones that have accumulated enough trust
        // This removes "ghost" cones that appeared for 1-2 frames and disappeared.
        if (cone.trust_score >= VALIDATION_THRESHOLD) {
            cone.is_validated = true;
            valid_cones.push_back(cone);
        }
    }
    return valid_cones;
}


} // namespace mapping
} // namespace fsai