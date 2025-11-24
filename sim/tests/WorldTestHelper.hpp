#pragma once

#include <Eigen/Dense>

#include "World.hpp"
#include "sim/mission/MissionDefinition.hpp"

class WorldTestHelper {
public:
    static void ConfigureSimpleGate(World& world) {
        fsai::sim::TrackData track{};
        track.checkpoints.push_back(MakeTransform(0.0f, 5.0f));
        track.checkpoints.push_back(MakeTransform(0.0f, 15.0f));
        track.leftCones.push_back(MakeTransform(-1.0f, 5.0f));
        track.rightCones.push_back(MakeTransform(1.0f, 5.0f));

        ConfigureTrack(world, track);
        world.initializeVehiclePose();
        world.insideLastCheckpoint_ = false;
    }

    static std::vector<Vector3> Checkpoints(const World& world) {
        return world.checkpointPositions;
    }

    static void SetPrev(World& world, Vector2 prev) {
        world.prevCarPos_ = prev;
    }

    static void SetCarPosition(World& world, VehicleDynamics& dynamics, float x, float z) {
        SetVehiclePose(world, dynamics, x, world.vehicleTransform().position.y, z);
    }

    static void SetCarHeight(World& world, VehicleDynamics& dynamics, float y) {
        const Transform& carTransform = world.vehicleTransform();
        SetVehiclePose(world, dynamics, carTransform.position.x, y, carTransform.position.z);
    }

    static bool CrossesGate(World& world, Vector2 prev, Vector2 curr) {
        return world.crossesCurrentGate(prev, curr);
    }

    static void ConfigureTrack(World& world, const fsai::sim::TrackData& track) {
        world.trackBuilderConfig_.vehicleCollisionRadius = world.config.vehicleCollisionRadius;
        TrackBuilder builder(world.trackBuilderConfig_);
        fsai::sim::MissionDefinition mission = world.mission_;
        mission.track = track;
        world.trackState_ = builder.Build(mission);
        world.mission_ = mission;
        world.configureTrackState(world.trackState_);
    }

    static void SetCollisionRadius(World& world, float radius) {
        world.config.vehicleCollisionRadius = radius;
        world.trackBuilderConfig_.vehicleCollisionRadius = radius;
    }

    static void SetInsideLastCheckpoint(World& world, bool inside) {
        world.insideLastCheckpoint_ = inside;
    }

private:
    static Transform MakeTransform(float x, float z) {
        Transform t{};
        t.position = Vector3{x, 0.0f, z};
        t.yaw = 0.0f;
        return t;
    }

    static void SetVehiclePose(World& world, VehicleDynamics& dynamics, float x, float y, float z) {
        VehicleState state = dynamics.state();
        state.position = Eigen::Vector3d(static_cast<double>(x), static_cast<double>(z), state.position.z());

        Transform transform = world.vehicleTransform();
        transform.position.x = x;
        transform.position.y = y;
        transform.position.z = z;

        dynamics.setState(state, transform);
    }
};
