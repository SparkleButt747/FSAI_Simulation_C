#pragma once

#include <vector>

#include "Transform.h"
#include "Vector.h"
#include "sim/mission/MissionDefinition.hpp"

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

struct CollisionSegment {
    Vector2 start{0.0f, 0.0f};
    Vector2 end{0.0f, 0.0f};
    float radius{0.0f};
    Vector2 boundsMin{0.0f, 0.0f};
    Vector2 boundsMax{0.0f, 0.0f};
};

struct TrackBuildResult {
    fsai::sim::TrackData track{};
    std::vector<Vector3> checkpointPositions{};
    std::vector<Cone> startCones{};
    std::vector<Cone> leftCones{};
    std::vector<Cone> rightCones{};
    std::vector<Vector3> startConePositions{};
    std::vector<Vector3> leftConePositions{};
    std::vector<Vector3> rightConePositions{};
    std::vector<CollisionSegment> gateSegments{};
    std::vector<CollisionSegment> boundarySegments{};
    Vector3 lastCheckpoint{0.0f, 0.0f, 0.0f};
};

