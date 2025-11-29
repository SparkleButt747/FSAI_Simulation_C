#pragma once

#include "World.hpp"
#include "sim/mission/MissionDefinition.hpp"

class WorldTestHelper {
public:
    static void ConfigureSimpleGate(World& world) {
        world.checkpointPositions.clear();
        world.checkpointPositions.push_back(Vector3{0.0f, 0.0f, 5.0f});
        world.checkpointPositions.push_back(Vector3{0.0f, 0.0f, 15.0f});

        world.leftCones.clear();
        Cone left{};
        left.position = Vector3{-1.0f, 0.0f, 5.0f};
        left.radius = 0.25f;
        left.mass = 1.0f;
        left.type = ConeType::Left;
        world.leftCones.push_back(left);

        world.rightCones.clear();
        Cone right{};
        right.position = Vector3{1.0f, 0.0f, 5.0f};
        right.radius = 0.25f;
        right.mass = 1.0f;
        right.type = ConeType::Right;
        world.rightCones.push_back(right);

        world.startCones.clear();
        world.gateSegments_.clear();
        world.boundarySegments_.clear();
        world.lastCheckpoint = Vector3{1000.0f, 0.0f, 1000.0f};
        world.prevCarPos_ = Vector2{0.0f, 0.0f};
        world.carTransform.position.x = 0.0f;
        world.carTransform.position.y = 0.5f;
        world.carTransform.position.z = 0.0f;
        world.insideLastCheckpoint_ = false;
    }

    static std::vector<Vector3> Checkpoints(const World& world) {
        return world.checkpointPositions;
    }

    static void SetPrev(World& world, Vector2 prev) {
        world.prevCarPos_ = prev;
    }

    static void SetCarPosition(World& world, float x, float z) {
        world.carTransform.position.x = x;
        world.carTransform.position.z = z;
    }

    static void SetCarHeight(World& world, float y) {
        world.carTransform.position.y = y;
    }

    static bool CrossesGate(World& world, Vector2 prev, Vector2 curr) {
        return world.crossesCurrentGate(prev, curr);
    }

    static void ConfigureTrack(World& world, const fsai::sim::TrackData& track) {
        world.configureTrackState(track);
    }

    static void SetCollisionRadius(World& world, float radius) {
        world.config.vehicleCollisionRadius = radius;
    }

    static void SetInsideLastCheckpoint(World& world, bool inside) {
        world.insideLastCheckpoint_ = inside;
    }
};

