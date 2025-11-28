#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

#include"common/types.h"

namespace fsai {
namespace vision {

struct MapCone {
    int id;
    double x;
    double y;
    double variance; 
    types::ConeSide side;
    
    // REPLACES simple observation_count
    // We sum up the confidence of all observations.
    // High confidence detections increase this faster.
    double trust_score = 0.0; 
    
    bool is_validated = false; // Has it passed the threshold to be sent to path planning?
};

struct CarState {
    double x, y, heading;
};

class BayesianMapper {
public:
    BayesianMapper();
    
    void updateMap(const std::vector<types::BoxBound>& observations, const CarState& car);
    
    /**
     *@brief Applies recursive bayesian updating to create a smooth map
     */
    std::vector<MapCone> getValidatedMap();
    void updateMap(const std::vector<FsaiConeDet>& global_detections, const CarState& car);

private:
    // --- Tunable Params ---
    const double ASSOCIATION_THRESH = 1.5;
    
    // Sensor Noise
    const double BASE_SENSOR_NOISE = 0.05;
    const double DIST_NOISE_FACTOR = 0.08; 
    
    // Pruning Logic
    const double VALIDATION_THRESHOLD = 2.5;

    std::vector<MapCone> map_cones_;
    int global_id_counter_ = 0;

    void transformToGlobal(const types::BoxBound& obs, const CarState& car, double& gx, double& gy);
    void bayesianUpdate(MapCone& cone, double meas_x, double meas_y, double meas_var);
};

}
}