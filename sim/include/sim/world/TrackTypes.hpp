#pragma once

#include "Vector.h"

enum class ConeType {
    Start,
    Left,
    Right,
    Orange,
};

struct Cone {
    Vector3 position{0.0f, 0.0f, 0.0f};
    float radius{0.0f};
    float mass{0.0f};
    ConeType type{ConeType::Left};
};

struct CollisionSegment {
    Vector2 start{0.0f, 0.0f};
    Vector2 end{0.0f, 0.0f};
    float radius{0.0f};
    Vector2 boundsMin{0.0f, 0.0f};
    Vector2 boundsMax{0.0f, 0.0f};
};
