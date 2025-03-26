#ifndef COLLISION_H
#define COLLISION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Vector.h"

    // Checks if the Euclidean distance between two 3D points is less than the given threshold.
    // Returns 1 if a collision is detected, 0 otherwise.
    int Collision_CheckPoint(const Vector3 *a, const Vector3 *b, float threshold);

#ifdef __cplusplus
}
#endif

#endif // COLLISION_H
