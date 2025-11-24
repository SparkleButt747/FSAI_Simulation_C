#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>

#include "World.hpp"
#include "WorldConfigLoader.hpp"
#include "mission/MissionDefinition.hpp"
#include "VehicleState.hpp"
#include "tests/WorldTestHelper.hpp"

namespace fs = std::filesystem;

namespace {
Transform MakeTransform(float x, float z) {
    Transform t{};
    t.position.x = x;
    t.position.y = 0.0f;
    t.position.z = z;
    t.yaw = 0.0f;
    return t;
}
}

TEST(WorldConfigLoaderTest, LoadsDefaultConfigFile) {
    fsai::sim::MissionDefinition mission{};
    const auto root = fs::path(__FILE__).parent_path().parent_path();
    const auto config_path = root / "configs/sim/world.yaml";
    const auto config = fsai::sim::WorldConfigLoader::FromFile(config_path.string(), mission);

    EXPECT_FLOAT_EQ(config.runtime.collisionThreshold, 1.75f);
    EXPECT_FLOAT_EQ(config.runtime.vehicleCollisionRadius, 0.386f);
    EXPECT_FLOAT_EQ(config.runtime.lapCompletionThreshold, 0.2f);
    EXPECT_FLOAT_EQ(config.runtime.speedLookAheadSensitivity, 0.5f);
    EXPECT_FLOAT_EQ(config.runtime.steeringLookAheadSensitivity, 0.0f);
    EXPECT_FLOAT_EQ(config.runtime.accelerationFactor, 0.0019f);
}

TEST(WorldConfigLoaderTest, AppliesConfigToWorldRuntime) {
    const fs::path temp_config = fs::temp_directory_path() / "world_config_test.yaml";
    std::ofstream out(temp_config);
    out << "collision_threshold: 3.25\n";
    out << "vehicle_collision_radius: 0.42\n";
    out << "lap_completion_threshold: 0.55\n";
    out << "controller:\n";
    out << "  speed_lookahead_sensitivity: 0.9\n";
    out << "  steering_lookahead_sensitivity: 1.1\n";
    out << "  acceleration_factor: 0.0033\n";
    out.close();

    fsai::sim::MissionDefinition mission{};
    mission.trackSource = fsai::sim::TrackSource::kCsv;
    mission.track.checkpoints = {MakeTransform(0.0f, 0.0f), MakeTransform(0.0f, 10.0f)};
    mission.track.leftCones = {MakeTransform(-1.0f, 0.0f), MakeTransform(-1.0f, 10.0f)};
    mission.track.rightCones = {MakeTransform(1.0f, 0.0f), MakeTransform(1.0f, 10.0f)};

    auto world_config = fsai::sim::WorldConfigLoader::FromFile(temp_config.string(), mission);

    VehicleDynamics dynamics;
    World world;
    world.init(dynamics, world_config);

    EXPECT_FLOAT_EQ(WorldTestHelper::CollisionThreshold(world), 3.25f);
    EXPECT_FLOAT_EQ(WorldTestHelper::VehicleCollisionRadius(world), 0.42f);
    EXPECT_FLOAT_EQ(WorldTestHelper::LapCompletionThreshold(world), 0.55f);

    const auto racing = WorldTestHelper::RacingControllerConfig(world);
    EXPECT_FLOAT_EQ(racing.speedLookAheadSensitivity, 0.9f);
    EXPECT_FLOAT_EQ(racing.steeringLookAheadSensitivity, 1.1f);
    EXPECT_FLOAT_EQ(racing.accelerationFactor, 0.0033f);

    fs::remove(temp_config);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

