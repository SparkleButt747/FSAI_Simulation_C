#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"

namespace {

Transform MakeCone(float x, float z) {
    Transform transform{};
    transform.position.x = x;
    transform.position.y = 0.0f;
    transform.position.z = z;
    transform.yaw = 0.0f;
    return transform;
}

bool AlmostEqual(float a, float b, float epsilon = 1e-3f) {
    return std::fabs(a - b) <= epsilon;
}

bool TestAsymmetricBoundaryMatching() {
    std::vector<Transform> left{
        MakeCone(0.0f, 0.0f),
        MakeCone(0.0f, 5.0f),
        MakeCone(0.0f, 10.0f),
    };
    std::vector<Transform> right{
        MakeCone(4.0f, 1.0f),
        MakeCone(4.0f, 6.0f),
        MakeCone(4.0f, 11.0f),
        MakeCone(4.0f, 16.0f),
    };

    const auto gates = MatchConeBoundaries(left, right);
    if (gates.size() != left.size()) {
        std::cerr << "Expected one gate per left cone" << std::endl;
        return false;
    }
    if (gates.size() != std::min(left.size(), right.size())) {
        std::cerr << "Gate count should match the smaller boundary size" << std::endl;
        return false;
    }

    std::vector<bool> right_used(right.size(), false);
    for (std::size_t i = 0; i < gates.size(); ++i) {
        const auto& gate = gates[i];
        const Transform& left_cone = left[i];
        if (!AlmostEqual(gate.first.position.x, left_cone.position.x) ||
            !AlmostEqual(gate.first.position.z, left_cone.position.z)) {
            std::cerr << "Left cone order should be preserved" << std::endl;
            return false;
        }

        bool matched = false;
        for (std::size_t r = 0; r < right.size(); ++r) {
            if (!AlmostEqual(gate.second.position.x, right[r].position.x) ||
                !AlmostEqual(gate.second.position.z, right[r].position.z)) {
                continue;
            }
            if (right_used[r]) {
                std::cerr << "Right cone matched more than once" << std::endl;
                return false;
            }
            right_used[r] = true;
            matched = true;
            break;
        }
        if (!matched) {
            std::cerr << "Gate did not reference an original right cone" << std::endl;
            return false;
        }

        const float mid_x = 0.5f * (gate.first.position.x + gate.second.position.x);
        const float mid_z = 0.5f * (gate.first.position.z + gate.second.position.z);
        const float left_dx = mid_x - gate.first.position.x;
        const float left_dz = mid_z - gate.first.position.z;
        const float right_dx = gate.second.position.x - mid_x;
        const float right_dz = gate.second.position.z - mid_z;
        const float left_dist = std::hypot(left_dx, left_dz);
        const float right_dist = std::hypot(right_dx, right_dz);
        if (!AlmostEqual(left_dist, right_dist)) {
            std::cerr << "Gate midpoint is not centered between cones" << std::endl;
            return false;
        }
    }

    if (std::count(right_used.begin(), right_used.end(), true) != static_cast<int>(gates.size())) {
        std::cerr << "Unexpected number of matched right cones" << std::endl;
        return false;
    }

    return true;
}

bool TestTrackResultGateConsistency() {
    PathConfig config;
    PathGenerator pathGen(config);
    PathResult path = pathGen.generatePath(config.resolution);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(config, path);

    if (track.gates.size() != track.checkpoints.size()) {
        std::cerr << "Checkpoints must align with gate count" << std::endl;
        return false;
    }
    if (track.gates.empty()) {
        std::cerr << "Track generation produced no gates" << std::endl;
        return false;
    }

    std::vector<bool> left_used(track.leftCones.size(), false);
    std::vector<bool> right_used(track.rightCones.size(), false);

    for (std::size_t i = 0; i < track.gates.size(); ++i) {
        const auto& gate = track.gates[i];
        const auto& checkpoint = track.checkpoints[i];

        const float expected_x = 0.5f * (gate.first.position.x + gate.second.position.x);
        const float expected_z = 0.5f * (gate.first.position.z + gate.second.position.z);
        if (!AlmostEqual(checkpoint.position.x, expected_x) ||
            !AlmostEqual(checkpoint.position.z, expected_z)) {
            std::cerr << "Checkpoint midpoint mismatch" << std::endl;
            return false;
        }
        const float dx = gate.second.position.x - gate.first.position.x;
        const float dz = gate.second.position.z - gate.first.position.z;
        const float expected_yaw = std::atan2(dz, dx);
        if (!AlmostEqual(checkpoint.yaw, expected_yaw)) {
            std::cerr << "Checkpoint yaw mismatch" << std::endl;
            return false;
        }

        bool left_found = false;
        for (std::size_t l = 0; l < track.leftCones.size(); ++l) {
            if (!AlmostEqual(gate.first.position.x, track.leftCones[l].position.x) ||
                !AlmostEqual(gate.first.position.z, track.leftCones[l].position.z)) {
                continue;
            }
            if (left_used[l]) {
                std::cerr << "Left cone reused in gates" << std::endl;
                return false;
            }
            left_used[l] = true;
            left_found = true;
            break;
        }
        if (!left_found) {
            std::cerr << "Gate left cone not found in track" << std::endl;
            return false;
        }

        bool right_found = false;
        for (std::size_t r = 0; r < track.rightCones.size(); ++r) {
            if (!AlmostEqual(gate.second.position.x, track.rightCones[r].position.x) ||
                !AlmostEqual(gate.second.position.z, track.rightCones[r].position.z)) {
                continue;
            }
            if (right_used[r]) {
                std::cerr << "Right cone reused in gates" << std::endl;
                return false;
            }
            right_used[r] = true;
            right_found = true;
            break;
        }
        if (!right_found) {
            std::cerr << "Gate right cone not found in track" << std::endl;
            return false;
        }
    }

    return true;
}

void MaybeWriteDebugOutputs() {
    const char* dump_env = std::getenv("FSAI_WRITE_TRACK_FILES");
    if (!dump_env || std::strcmp(dump_env, "1") != 0) {
        return;
    }

    std::srand(static_cast<unsigned>(std::time(nullptr)));
    PathConfig config;
    int nPoints = config.resolution;
    PathGenerator pathGen(config);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(config, path);

    std::ofstream fp("left_cones.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.leftCones) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    fp.open("right_cones.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.rightCones) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    fp.open("start_cones.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.startCones) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    fp.open("checkpoints.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.checkpoints) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    std::cout << "Data written to left_cones.csv, right_cones.csv, start_cones.csv, and checkpoints.csv" << std::endl;
    int ret = std::system("python ../PathLogic/plot_track.py");
    if (ret != 0) {
        std::cerr << "Graphing tool did not run successfully. Please run plot_track.py manually." << std::endl;
    }
}

}  // namespace

int main() {
    if (!TestAsymmetricBoundaryMatching()) {
        return 1;
    }
    if (!TestTrackResultGateConsistency()) {
        return 1;
    }

    MaybeWriteDebugOutputs();
    return 0;
}

