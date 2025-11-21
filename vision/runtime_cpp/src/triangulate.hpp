#pragma once

#include "common/include/common/types.h"
#include <cmath>
#include <limits>
namespace fsai{
namespace vision{
struct Point2D{
    double x,y;
};

struct Point3D{
    double X,Y,Z;
};

bool triangulate(const Point2D& p1, const Point2D& p2,
                 const FsaiCameraExtrinsics& params,
                 Point3D& result);

class StereoTriangulation{
    StereoTriangulation();
    ~StereoTriangulation();

    public:
        inline bool triangulatePoint(const Point2D& p1, const Point2D& p2, Point3D& result);
    private:
        FsaiCameraIntrinsics cameraParams_;
        const double BASE_LINE_ {0.12};
};
}
}