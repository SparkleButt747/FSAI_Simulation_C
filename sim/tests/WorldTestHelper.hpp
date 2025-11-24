#pragma once

#include <Eigen/Dense>

#include "World.hpp"
#include "MissionDefinition.hpp"
#include "TrackBuilder.hpp"

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
        world.initializeVehiclePose();
        world.insideLastCheckpoint_ = false;
    }

    static WorldVehicleContext MakeContext(VehicleDynamics& dynamics) {
        WorldVehicleContext context{};
        context.dynamics = &dynamics;
        context.dynamics_model = &dynamics.model();
        context.reset_vehicle = [&dynamics](const VehicleSpawnState& spawn) {
            dynamics.setState(spawn.state, spawn.transform);
        };
        return context;
    }

    static std::vector<Vector3> Checkpoints(const World& world) {
        return world.checkpointPositions;
    }

    static void SetPrev(World& world, Vector2 prev) {
        world.prevCarPos_ = prev;
    }

    static void SetCarPosition(World& world, VehicleDynamics& dynamics, float x, float z) {
        SetVehiclePose(world, dynamics, x, world.vehicle_transform().position.y, z);
    }

    static void SetCarHeight(World& world, VehicleDynamics& dynamics, float y) {
        const Transform& carTransform = world.vehicle_transform();
        SetVehiclePose(world, dynamics, carTransform.position.x, y, carTransform.position.z);
    }

    static bool CrossesGate(World& world, Vector2 prev, Vector2 curr) {
        return world.crossesCurrentGate(prev, curr);
    }

    static void ConfigureTrack(World& world, const fsai::sim::TrackData& track) {
        TrackBuilder builder;
        fsai::sim::MissionDefinition mission = world.mission_;
        mission.track = track;
        mission.trackSource = fsai::sim::TrackSource::kCsv;
        const TrackBuildResult trackState =
            builder.Build(mission, world.pathConfig_, world.config.vehicleCollisionRadius);
        world.trackState_ = trackState;
        world.mission_ = mission;
        world.configureTrackState(trackState);
    }

    static void SetCollisionRadius(World& world, float radius) {
        world.config.vehicleCollisionRadius = radius;
    }

    static void SetInsideLastCheckpoint(World& world, bool inside) {
        world.insideLastCheckpoint_ = inside;
    }

private:
    static void SetVehiclePose(World& world, VehicleDynamics& dynamics, float x, float y, float z) {
        VehicleState state = dynamics.state();
        state.position = Eigen::Vector3d(static_cast<double>(x), static_cast<double>(z), state.position.z());

        Transform transform = world.vehicle_transform();
        transform.position.x = x;
        transform.position.y = y;
        transform.position.z = z;

        dynamics.setState(state, transform);
    }
};

