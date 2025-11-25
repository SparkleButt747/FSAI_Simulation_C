#pragma once

#include <optional>
#include <vector>

#include "sim/WorldRuntime.hpp"
#include "sim/world/TrackTypes.hpp"
#include "Vector.h"

struct CollisionResult {
    bool crossedGate{false};
    bool coneCollision{false};
    bool boundaryCollision{false};
    std::optional<fsai::sim::WorldRuntime::ResetReason> resetReason{};
};

class CollisionService {
public:
    struct Config {
        float vehicleCollisionRadius{0.386f};
        float gateThreshold{2.5f};
    };

    explicit CollisionService(Config config);

    CollisionResult Evaluate(const Vector2& previousPosition, const Vector2& currentPosition,
                             const std::vector<Vector3>& checkpointPositions,
                             const std::vector<Cone>& startCones,
                             const std::vector<Cone>& leftCones,
                             const std::vector<Cone>& rightCones,
                             const std::vector<CollisionSegment>& gateSegments,
                             const std::vector<CollisionSegment>& boundarySegments) const;

private:
    bool crossesCurrentGate(const Vector2& previous, const Vector2& current,
                            const std::vector<Vector3>& checkpointPositions,
                            const std::vector<Cone>& leftCones,
                            const std::vector<Cone>& rightCones) const;
    bool collidesWithCones(const Vector2& position, const std::vector<Cone>& cones) const;
    bool collidesWithSegments(const Vector2& position,
                              const std::vector<CollisionSegment>& segments) const;

    Config config_{};
};

