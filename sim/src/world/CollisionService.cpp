#include "CollisionService.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Transform.h"
#include "Vector.h"
#include "World.hpp"

namespace {

float distanceSquaredToSegment(const Vector2& point, const CollisionSegment& segment) {
    const float ax = segment.start.x;
    const float ay = segment.start.y;
    const float bx = segment.end.x;
    const float by = segment.end.y;
    const float abx = bx - ax;
    const float aby = by - ay;
    const float lengthSq = abx * abx + aby * aby;
    if (lengthSq <= std::numeric_limits<float>::epsilon()) {
        const float dx = point.x - ax;
        const float dy = point.y - ay;
        return dx * dx + dy * dy;
    }

    const float t = ((point.x - ax) * abx + (point.y - ay) * aby) / lengthSq;
    const float clamped = std::clamp(t, 0.0f, 1.0f);
    const float closestX = ax + clamped * abx;
    const float closestY = ay + clamped * aby;
    const float dx = point.x - closestX;
    const float dy = point.y - closestY;
    return dx * dx + dy * dy;
}

bool pointWithinBounds(const Vector2& point, const CollisionSegment& segment) {
    return point.x >= segment.boundsMin.x && point.x <= segment.boundsMax.x &&
           point.y >= segment.boundsMin.y && point.y <= segment.boundsMax.y;
}

}  // namespace

CollisionService::CollisionService(const Config& config,
                                   const std::vector<Cone>& startCones,
                                   const std::vector<Cone>& leftCones,
                                   const std::vector<Cone>& rightCones,
                                   const std::vector<CollisionSegment>& gateSegments,
                                   const std::vector<CollisionSegment>& boundarySegments,
                                   float vehicleCollisionRadius)
    : config_(config),
      startCones_(startCones),
      leftCones_(leftCones),
      rightCones_(rightCones),
      gateSegments_(gateSegments),
      boundarySegments_(boundarySegments),
      vehicleCollisionRadius_(vehicleCollisionRadius) {}

CollisionService::Result CollisionService::Evaluate(const Transform& carTransform,
                                                    const Vector3& lastCheckpoint,
                                                    bool wasInsideLapZone) const {
    const Vector2 carCenter{carTransform.position.x, carTransform.position.z};

    Result result{};
    result.insideLapZone = insideLapCompletionZone(carCenter, lastCheckpoint);
    result.lapCompleted = result.insideLapZone && !wasInsideLapZone;
    result.coneCollision = collidesWithCones(carCenter);
    result.boundaryCollision = collidesWithBoundaries(carCenter);

    return result;
}

bool CollisionService::collidesWithCones(const Vector2& carCenter) const {
    const auto coneHit = [&](const Cone& cone) {
        const float dx = carCenter.x - cone.position.x;
        const float dz = carCenter.y - cone.position.z;
        const float distance = std::sqrt(dx * dx + dz * dz);
        const float combinedRadius = cone.radius + vehicleCollisionRadius_;
        return distance < combinedRadius;
    };

    for (const auto& cone : startCones_) {
        if (coneHit(cone)) {
            return true;
        }
    }
    for (const auto& cone : leftCones_) {
        if (coneHit(cone)) {
            return true;
        }
    }
    for (const auto& cone : rightCones_) {
        if (coneHit(cone)) {
            return true;
        }
    }
    return false;
}

bool CollisionService::collidesWithBoundaries(const Vector2& carCenter) const {
    const float collisionRadiusSq = vehicleCollisionRadius_ * vehicleCollisionRadius_;

    auto segmentHit = [&](const CollisionSegment& segment) {
        if (!pointWithinBounds(carCenter, segment)) {
            return false;
        }
        return distanceSquaredToSegment(carCenter, segment) < collisionRadiusSq;
    };

    for (const auto& segment : gateSegments_) {
        if (segmentHit(segment)) {
            break;
        }
    }

    for (const auto& segment : boundarySegments_) {
        if (segmentHit(segment)) {
            return true;
        }
    }

    return false;
}

bool CollisionService::insideLapCompletionZone(const Vector2& carCenter,
                                               const Vector3& lastCheckpoint) const {
    const float dx = carCenter.x - lastCheckpoint.x;
    const float dz = carCenter.y - lastCheckpoint.z;
    const float distToLast = std::sqrt(dx * dx + dz * dz);
    return distToLast < config_.lapCompletionThreshold;
}

