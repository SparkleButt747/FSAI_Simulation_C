#include "CollisionService.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace {

struct Vec2d {
    double x{0.0};
    double y{0.0};
};

constexpr double kEpsilon = 1e-6;

double cross(const Vec2d& a, const Vec2d& b) { return a.x * b.y - a.y * b.x; }

double orientation(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
    return cross({b.x - a.x, b.y - a.y}, {c.x - a.x, c.y - a.y});
}

bool onSegment(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
    const double minX = std::min(a.x, b.x) - kEpsilon;
    const double maxX = std::max(a.x, b.x) + kEpsilon;
    const double minY = std::min(a.y, b.y) - kEpsilon;
    const double maxY = std::max(a.y, b.y) + kEpsilon;
    return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY;
}

bool segmentsIntersect(const Vec2d& p1, const Vec2d& p2, const Vec2d& q1, const Vec2d& q2) {
    const double o1 = orientation(p1, p2, q1);
    const double o2 = orientation(p1, p2, q2);
    const double o3 = orientation(q1, q2, p1);
    const double o4 = orientation(q1, q2, p2);

    const bool generalCase = ((o1 > kEpsilon && o2 < -kEpsilon) || (o1 < -kEpsilon && o2 > kEpsilon)) &&
                             ((o3 > kEpsilon && o4 < -kEpsilon) || (o3 < -kEpsilon && o4 > kEpsilon));
    if (generalCase) {
        return true;
    }

    if (std::abs(o1) <= kEpsilon && onSegment(p1, p2, q1)) {
        return true;
    }
    if (std::abs(o2) <= kEpsilon && onSegment(p1, p2, q2)) {
        return true;
    }
    if (std::abs(o3) <= kEpsilon && onSegment(q1, q2, p1)) {
        return true;
    }
    if (std::abs(o4) <= kEpsilon && onSegment(q1, q2, p2)) {
        return true;
    }

    return false;
}

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

CollisionService::CollisionService(Config config) : config_(config) {}

CollisionResult CollisionService::Evaluate(
    const Vector2& previousPosition, const Vector2& currentPosition,
    const std::vector<Vector3>& checkpointPositions, const std::vector<Cone>& startCones,
    const std::vector<Cone>& leftCones, const std::vector<Cone>& rightCones,
    const std::vector<CollisionSegment>& gateSegments,
    const std::vector<CollisionSegment>& boundarySegments) const {
    CollisionResult result{};
    result.crossedGate = crossesCurrentGate(previousPosition, currentPosition, checkpointPositions,
                                            leftCones, rightCones);

    if (collidesWithCones(currentPosition, startCones) || collidesWithCones(currentPosition, leftCones) ||
        collidesWithCones(currentPosition, rightCones)) {
        result.coneCollision = true;
        result.resetReason = fsai::sim::WorldRuntime::ResetReason::kConeCollision;
        return result;
    }

    if (collidesWithSegments(currentPosition, gateSegments)) {
        // Crossing through the gate is allowed; collisions here simply acknowledge passage.
    }

    if (collidesWithSegments(currentPosition, boundarySegments)) {
        result.boundaryCollision = true;
        result.resetReason = fsai::sim::WorldRuntime::ResetReason::kBoundaryCollision;
    }

    return result;
}

bool CollisionService::crossesCurrentGate(const Vector2& previous, const Vector2& current,
                                          const std::vector<Vector3>& checkpointPositions,
                                          const std::vector<Cone>& leftCones,
                                          const std::vector<Cone>& rightCones) const {
    const Vec2d prev{static_cast<double>(previous.x), static_cast<double>(previous.y)};
    const Vec2d curr{static_cast<double>(current.x), static_cast<double>(current.y)};

    if (leftCones.empty() || rightCones.empty()) {
        if (checkpointPositions.empty()) {
            return false;
        }
        const Vector3& checkpoint = checkpointPositions.front();
        const Vec2d cp{static_cast<double>(checkpoint.x), static_cast<double>(checkpoint.z)};
        const double prevDist = std::hypot(prev.x - cp.x, prev.y - cp.y);
        const double currDist = std::hypot(curr.x - cp.x, curr.y - cp.y);
        return currDist < static_cast<double>(config_.gateThreshold) &&
               prevDist >= static_cast<double>(config_.gateThreshold);
    }

    const Vec2d left{static_cast<double>(leftCones.front().position.x),
                     static_cast<double>(leftCones.front().position.z)};
    const Vec2d right{static_cast<double>(rightCones.front().position.x),
                      static_cast<double>(rightCones.front().position.z)};

    const double radius = static_cast<double>(config_.vehicleCollisionRadius);
    const double minX = std::min(left.x, right.x) - radius;
    const double maxX = std::max(left.x, right.x) + radius;
    const double minY = std::min(left.y, right.y) - radius;
    const double maxY = std::max(left.y, right.y) + radius;

    if ((prev.x < minX && curr.x < minX) || (prev.x > maxX && curr.x > maxX) ||
        (prev.y < minY && curr.y < minY) || (prev.y > maxY && curr.y > maxY)) {
        return false;
    }

    const Vec2d gateVector{right.x - left.x, right.y - left.y};
    const double gateLenSq = gateVector.x * gateVector.x + gateVector.y * gateVector.y;
    if (gateLenSq <= kEpsilon) {
        return false;
    }

    const double gateLength = std::sqrt(gateLenSq);
    const double margin = gateLength > kEpsilon ? radius / gateLength : 0.0;
    const double prevProj = ((prev.x - left.x) * gateVector.x + (prev.y - left.y) * gateVector.y) / gateLenSq;
    const double currProj = ((curr.x - left.x) * gateVector.x + (curr.y - left.y) * gateVector.y) / gateLenSq;
    const double minProj = -margin;
    const double maxProj = 1.0 + margin;
    if ((prevProj < minProj && currProj < minProj) || (prevProj > maxProj && currProj > maxProj)) {
        return false;
    }

    const double oriPrev = orientation(left, right, prev);
    const double oriCurr = orientation(left, right, curr);
    const bool signChange = (oriPrev > kEpsilon && oriCurr < -kEpsilon) ||
                            (oriPrev < -kEpsilon && oriCurr > kEpsilon);
    if (signChange) {
        return true;
    }

    if (segmentsIntersect(prev, curr, left, right)) {
        return true;
    }

    if (std::abs(oriPrev) <= kEpsilon && onSegment(left, right, prev)) {
        return true;
    }
    if (std::abs(oriCurr) <= kEpsilon && onSegment(left, right, curr)) {
        return true;
    }

    return false;
}

bool CollisionService::collidesWithCones(const Vector2& position, const std::vector<Cone>& cones) const {
    for (const auto& cone : cones) {
        float cdx = position.x - cone.position.x;
        float cdz = position.y - cone.position.z;
        float cdist = std::sqrt(cdx * cdx + cdz * cdz);
        const float combinedRadius = cone.radius + config_.vehicleCollisionRadius;
        if (cdist < combinedRadius) {
            return true;
        }
    }
    return false;
}

bool CollisionService::collidesWithSegments(const Vector2& position,
                                            const std::vector<CollisionSegment>& segments) const {
    const float collisionRadiusSq = config_.vehicleCollisionRadius * config_.vehicleCollisionRadius;
    for (const auto& segment : segments) {
        if (!pointWithinBounds(position, segment)) {
            continue;
        }

        if (distanceSquaredToSegment(position, segment) < collisionRadiusSq) {
            return true;
        }
    }

    return false;
}

