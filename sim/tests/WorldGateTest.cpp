#include <gtest/gtest.h>

#include "World.hpp"

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

    static bool CrossesGate(World& world, Vector2 prev, Vector2 curr) {
        return world.crossesCurrentGate(prev, curr);
    }
};

TEST(WorldGateTest, AdvancesCheckpointOnCrossing) {
    World world;
    WorldTestHelper::ConfigureSimpleGate(world);
    const auto initial = WorldTestHelper::Checkpoints(world);

    const Vector2 prev{0.0f, 4.0f};
    const Vector2 curr{0.0f, 6.0f};
    WorldTestHelper::SetPrev(world, prev);
    WorldTestHelper::SetCarPosition(world, curr.x, curr.y);

    const bool crossed = WorldTestHelper::CrossesGate(world, prev, curr);
    EXPECT_TRUE(crossed);
    EXPECT_TRUE(world.detectCollisions(crossed));

    const auto after = WorldTestHelper::Checkpoints(world);
    EXPECT_EQ(initial.size(), after.size());
    EXPECT_EQ(initial[1].z, after[0].z);
}

TEST(WorldGateTest, GateIgnoresNonCrossingMotion) {
    World world;
    WorldTestHelper::ConfigureSimpleGate(world);
    const auto initial = WorldTestHelper::Checkpoints(world);

    const Vector2 prev{3.0f, 4.0f};
    const Vector2 curr{3.0f, 6.0f};
    WorldTestHelper::SetPrev(world, prev);
    WorldTestHelper::SetCarPosition(world, curr.x, curr.y);

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

