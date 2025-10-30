#ifndef TRANSFORM_H
#define TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <common/math/Vector.h>

// A simple transform: a 3D position and a yaw rotation (in radians)
typedef struct {
    Vector3 position;
    float yaw;
} Transform;

#ifdef __cplusplus
}
#endif

#endif // TRANSFORM_H
