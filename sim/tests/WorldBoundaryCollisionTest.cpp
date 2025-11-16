#include "gtest/gtest.h"

#include "WorldTestHelper.hpp"
#include "sim/cone_constants.hpp"

namespace {

Transform MakeTransform(float x, float z) {
    Transform t{};
    t.position = Vector3{x, 0.0f, z};
    t.yaw = 0.0f;
    return t;
}

}  // namespace

TEST(WorldBoundaryCollisionTest, LeftBoundaryTriggersReset) {
    World world;

    fsai::sim::TrackData track;
    track.leftCones.push_back(MakeTransform(-1.0f, 0.0f));
    track.leftCones.push_back(MakeTransform(-1.0f, 10.0f));
    track.rightCones.push_back(MakeTransform(1.0f, 0.0f));
    track.rightCones.push_back(MakeTransform(1.0f, 10.0f));
    track.checkpoints.push_back(MakeTransform(0.0f, 0.0f));
    track.checkpoints.push_back(MakeTransform(0.0f, 10.0f));

    WorldTestHelper::SetCollisionRadius(world, 0.5f - fsai::sim::kSmallConeRadiusMeters);
    WorldTestHelper::ConfigureTrack(world, track);
    WorldTestHelper::SetPrev(world, Vector2{0.0f, 5.0f});
    WorldTestHelper::SetCarPosition(world, -1.35f, 5.0f);
    WorldTestHelper::SetCarHeight(world, 0.5f);
    WorldTestHelper::SetInsideLastCheckpoint(world, false);

    EXPECT_FALSE(world.detectCollisions(false));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

