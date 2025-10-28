#include <iostream>
#include <cmath>
#include <limits>

#include "triangulate.hpp"
// #include "common/include/common/types.h"


struct Point2D{
    double x,y;
};

struct Point3D{
    double X,Y,Z;
};

struct StereoParams{
    double fLength;
    double bLine;
    double cx;
    double cy;
};
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
inline bool triangulate(const Point2D& p1, Point2D& p2,
                    const StereoParams& params,
                    Point3D& result){
    const double disparity = p1.x - p2.x;
    
    if(std::abs(disparity)< 1e-6){
        result = {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()

        };
        return false;
    }

    const double inv_disparity = 1.0/disparity;

    result.Z = params.bLine * params.fLength * inv_disparity;

    const double x1_norm = p1.x - params.cx;
    result.X = x1_norm * result.Z * inv_disparity;
    
    const double y1_norm = p1.x - params.cy;
    result.Y = y1_norm * result.Z * inv_disparity;

    return true;
}

int main(){
    //some local checks for sanity 

    StereoParams params = {
        500.0,
        0.12,
        320.0,
        240.0
    };

    Point2D cam1_point = {420.0, 250.0};
    Point2D cam2_point = {370.0, 250.0};

    Point3D coord_3d;

    if(triangulate(cam1_point,cam2_point,params,coord_3d)){
        std::cout << "Success in triangulating point" << std::endl;
        std::cout << "X: " << coord_3d.X << " m" << std::endl;
        std::cout << "Y: " << coord_3d.Y << " m" << std::endl;
        std::cout << "Z: " << coord_3d.Z << " m (Depth)" << std::endl;
    }else{
        std::cout << "Disparity is zero, unable to recover depth!" << std ::endl;
    }

    return 0; 

}