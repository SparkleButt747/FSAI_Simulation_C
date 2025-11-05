#include <iostream>
#include <cmath>
#include <limits>

#include "vision/triangulate.hpp"
// #include "common/include/common/types.h"

/**
 * Calculates the 3D coordinates of a point given its 2D projections
 * from two rectified stereo cameras.
 *
 * @param p1 The 2D point (in pixels) from the first camera (e.g., left).
 * @param p2 The 2D point (in pixels) from the second camera (e.g., right).
 * @param params Basic camera parameters needed for triangulation
 * @return A Point3D struct containing the (X, Y, Z) coordinates.
 */
// }
namespace fsai{
namespace vision{
    StereoTriangulation::StereoTriangulation(){};
    StereoTriangulation::~StereoTriangulation() = default;

    inline bool StereoTriangulation::triangulatePoint(const Point2D& p1, const Point2D& p2,Point3D&result){
    const double disparity = p1.x - p2.x;

    if (std::abs(disparity) < 1e-6) {
        result = { /* ... return infinity ... */ };
        return false;
    }

    const double inv_disparity = 1.0 / disparity;

    // --- Use fx for Z calculation, as disparity is on the x-axis ---
    // Z = (baseline * fx) / disparity
    result.Z = BASE_LINE_ * cameraParams_.fx * inv_disparity;

    // --- Use fx for X and fy for Y ---
    
    // X = (x - cx) * Z / fx
    const double x1_norm = p1.x - cameraParams_.cx;
    result.X = x1_norm * result.Z / cameraParams_.fx;
    
    // Y = (y - cy) * Z / fy
    const double y1_norm = p1.y - cameraParams_.cy ;
    result.Y = y1_norm * result.Z / cameraParams_.fy;

    return true;
    }
}
}