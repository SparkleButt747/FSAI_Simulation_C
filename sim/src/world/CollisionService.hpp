#pragma once

#include <vector>

struct Vector2;
struct Vector3;
struct Transform;
struct Cone;
struct CollisionSegment;

class CollisionService {
 public:
  struct Config {
    float lapCompletionThreshold{0.2f};
  };

  CollisionService(const Config& config,
                   const std::vector<Cone>& startCones,
                   const std::vector<Cone>& leftCones,
                   const std::vector<Cone>& rightCones,
                   const std::vector<CollisionSegment>& gateSegments,
                   const std::vector<CollisionSegment>& boundarySegments,
                   float vehicleCollisionRadius);

  struct Result {
    bool coneCollision{false};
    bool boundaryCollision{false};
    bool lapCompleted{false};
    bool insideLapZone{false};
  };

  Result Evaluate(const Transform& carTransform,
                  const Vector3& lastCheckpoint,
                  bool wasInsideLapZone) const;

 private:
  bool collidesWithCones(const Vector2& carCenter) const;
  bool collidesWithBoundaries(const Vector2& carCenter) const;
  bool insideLapCompletionZone(const Vector2& carCenter,
                               const Vector3& lastCheckpoint) const;

  const Config config_;
  const std::vector<Cone>& startCones_;
  const std::vector<Cone>& leftCones_;
  const std::vector<Cone>& rightCones_;
  const std::vector<CollisionSegment>& gateSegments_;
  const std::vector<CollisionSegment>& boundarySegments_;
  float vehicleCollisionRadius_{0.0f};
};

