#pragma once

#include <cmath>
#include <limits>

struct Point2D{
    double x,y;
};

struct Point3D{
    double X,Y,Z;
};

struct StereoParams{
    double fLength;
    double baseline;
    double cx;
    double cy;
};

bool triangulate(const Point2D& p1, const Point2D& p2,
                 const StereoParams& params,
                 Point3D& result);