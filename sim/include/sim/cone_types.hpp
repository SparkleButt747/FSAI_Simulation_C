#pragma once

#include "Vector.h"

enum class ConeType {
    Start,
    Left,
    Right,
};

struct Cone {
    Vector3 position{0.0f, 0.0f, 0.0f};
    float radius{0.0f};
    float mass{0.0f};
    ConeType type{ConeType::Left};
};
