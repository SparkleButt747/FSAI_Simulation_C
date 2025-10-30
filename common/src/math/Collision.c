#include <common/math/Collision.h>
#include <math.h>

int Collision_CheckPoint(const Vector3* a, const Vector3* b, float threshold) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    return (distance < threshold) ? 1 : 0;
}
