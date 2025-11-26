#include "gtest/gtest.h"

#include "WorldTestHelper.hpp"

TEST(WorldGateTest, AdvancesCheckpointOnCrossing) {
    World world;
    VehicleDynamics dynamics;
    world.setVehicleDynamics(dynamics);
    WorldTestHelper::ConfigureSimpleGate(world);
    const auto initial = WorldTestHelper::Checkpoints(world);

    const Vector2 prev{0.0f, 4.0f};
    const Vector2 curr{0.0f, 6.0f};
    WorldTestHelper::SetPrev(world, prev);
    WorldTestHelper::SetCarPosition(world, dynamics, curr.x, curr.y);

    const bool crossed = WorldTestHelper::CrossesGate(world, prev, curr);
    EXPECT_TRUE(crossed);
    EXPECT_TRUE(world.detectCollisions(crossed));

    const auto after = WorldTestHelper::Checkpoints(world);
    EXPECT_EQ(initial.size(), after.size());
    EXPECT_EQ(initial[1].z, after[0].z);
}

TEST(WorldGateTest, GateIgnoresNonCrossingMotion) {
    World world;
    VehicleDynamics dynamics;
    world.setVehicleDynamics(dynamics);
    WorldTestHelper::ConfigureSimpleGate(world);
    const auto initial = WorldTestHelper::Checkpoints(world);

    const Vector2 prev{3.0f, 4.0f};
    const Vector2 curr{3.0f, 6.0f};
    WorldTestHelper::SetPrev(world, prev);
    WorldTestHelper::SetCarPosition(world, dynamics, curr.x, curr.y);

    const bool crossed = WorldTestHelper::CrossesGate(world, prev, curr);
    EXPECT_FALSE(crossed);
    EXPECT_TRUE(world.detectCollisions(crossed));

    const auto after = WorldTestHelper::Checkpoints(world);
    EXPECT_EQ(initial[0].z, after[0].z);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

