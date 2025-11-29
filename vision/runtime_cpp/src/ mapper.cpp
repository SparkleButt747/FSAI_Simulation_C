#include "mapper.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace fsai {
namespace vision {

SimpleMap::SimpleMap() = default;
SimpleMap::~SimpleMap() = default;

void SimpleMap::update(const std::vector<FsaiConeDet>& new_detections, const Eigen::Vector2d& car_pos) {
    
    for (const auto& det : new_detections) {
        
        // 1. Calculate Weight for this NEW detection
        // Factor A: YOLO Confidence (Assuming det.conf is populated [0..1])
        float w_conf = det.conf; 

        // Factor B: Proximity Weighting
        // We trust detections closer to the car significantly more than far ones.
        // det.z is typically the Forward/North coordinate in global space here.
        float dx_car = det.x - (float)car_pos.x();
        float dy_car = det.z - (float)car_pos.y(); // Input Z maps to Global Y
        float dist_from_car = std::sqrt(dx_car*dx_car + dy_car*dy_car);
        
        // Weight formula: 1.0 at 0m, 0.5 at REF_DIST (10m)
        float w_prox = REF_DIST / (REF_DIST + dist_from_car); 

        // Combined weight
        float weight_new = w_conf * w_prox;

        // 2. Find Closest Existing Cone (Euclidean Distance Filter)
        int best_idx = -1;
        float best_dist = MERGE_RADIUS;

        for (size_t i = 0; i < cones.size(); ++i) {
            float dx = cones[i].x - det.x;
            float dy = cones[i].y - det.z; // Compare Map Y vs Input Z
            float dist = std::sqrt(dx*dx + dy*dy);

            // Simple Euclidean check
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = (int)i;
            }
        }

        // 3. Merge or Add
        if (best_idx != -1) {
            // --- MERGE (Weighted Average) ---
            MapCone& map_c = cones[best_idx];

            float total_weight = map_c.conf + weight_new;

            // Update Position: Weighted Average
            // New_Pos = (Old_Pos * Old_Weight + New_Pos * New_Weight) / Total
            map_c.x = (map_c.x * map_c.conf + det.x * weight_new) / total_weight;
            map_c.y = (map_c.y * map_c.conf + det.z * weight_new) / total_weight;
            
            // Update Weight/Confidence (capped)
            map_c.conf = std::min(map_c.conf + weight_new, MAX_CONF_CAP);
            map_c.seen_count++;

            // Optional: Update color logic? 
            // Often we just trust the existing color to prevent flickering
            // unless the new confidence is overwhelming.
        } else {
            // --- ADD NEW ---
            MapCone new_c;
            new_c.x = det.x;
            new_c.y = det.z; // Map Input Z -> Map Y
            new_c.side = det.side;
            new_c.conf = weight_new; // Initialize with calculated weight
            new_c.seen_count = 1;
            
            cones.push_back(new_c);
        }
    }
}

/**
 * @brief clears the current map
 */
void SimpleMap::clearMap(){cones.clear();}
} // namespace vision
} // namespace fsai