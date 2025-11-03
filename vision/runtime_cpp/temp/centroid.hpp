#pragma once

#include <vector>
#include <Eigen/Core>

namespace Centroid {
    // Constants for cone
    inline const double CONE_H = 0.325;
    inline const double CONE_R = 0.114;
    inline const double K = 2.850877; // K = CONE_H / CONE_R

    /**
     * @brief Linear least squares solution
     */
    Eigen::Vector2d centroid_linear(const std::vector<Eigen::Vector3d>& points);


    /**
     * @brief Non-linear LM optimization for precise solution
     * @param points The set of 3D point samples
     * @param initial Initial guess [cx, cy] (from linear solution)
     */
    Eigen::Vector2d centroid_nonlinear(const std::vector<Eigen::Vector3d>& points,
                                     const Eigen::Vector2d& initial);
}