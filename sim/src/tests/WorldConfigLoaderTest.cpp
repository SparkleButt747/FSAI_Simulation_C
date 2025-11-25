#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "sim/WorldConfig.hpp"
#include "sim/WorldRuntime.hpp"

namespace {

bool AlmostEqual(float a, float b, float epsilon = 1e-6f) {
    return std::fabs(a - b) <= epsilon;
}

bool TestLoadsDefaultConfig() {
    const std::string configPath = std::string(FSAI_PROJECT_ROOT) + "/configs/sim/world.yaml";
    fsai::sim::MissionDefinition mission;

    WorldConfig config = LoadWorldConfig(configPath, mission);

    if (!AlmostEqual(config.tuning.collisionThreshold, 1.75f) ||
        !AlmostEqual(config.tuning.vehicleCollisionRadius, 0.386f) ||
        !AlmostEqual(config.tuning.lapCompletionThreshold, 0.2f)) {
        std::cerr << "Tuning defaults were not loaded from config" << std::endl;
        return false;
    }

    if (!AlmostEqual(config.control.speedLookAheadSensitivity, 0.5f) ||
        !AlmostEqual(config.control.steeringLookAheadSensitivity, 0.0f) ||
        !AlmostEqual(config.control.accelerationFactor, 0.0019f)) {
        std::cerr << "Controller sensitivities were not loaded from config" << std::endl;
        return false;
    }

    if (config.control.pathSearchMaxLength != 30 || config.control.pathSearchMinLength != 2 ||
        config.control.pathSearchBeamWidth != 20) {
        std::cerr << "Path search parameters were not loaded from config" << std::endl;
        return false;
    }

    return true;
}

bool TestInvalidConfigThrows() {
    std::filesystem::path tempFile =
        std::filesystem::temp_directory_path() / "invalid_world_config.yaml";
    std::ofstream out(tempFile);
    out << "tuning:\n";
    out << "  collision_threshold: -1.0\n";
    out << "  vehicle_collision_radius: 0.386\n";
    out << "  lap_completion_threshold: 0.2\n";
    out << "control:\n";
    out << "  use_controller: true\n";
    out << "  regenerate_track: true\n";
    out << "  speed_lookahead_sensitivity: 0.5\n";
    out << "  steering_lookahead_sensitivity: 0.0\n";
    out << "  acceleration_factor: 0.0019\n";
    out << "  path_search_max_length: 30\n";
    out << "  path_search_min_length: 2\n";
    out << "  path_search_beam_width: 20\n";
    out.close();

    fsai::sim::MissionDefinition mission;
    try {
        (void)LoadWorldConfig(tempFile.string(), mission);
        std::cerr << "Expected LoadWorldConfig to throw for invalid tuning" << std::endl;
        return false;
    } catch (const std::exception&) {
        return true;
    }
}

bool TestWorldRuntimeUsesConfiguredLapMargin() {
    fsai::sim::WorldRuntime runtime;
    fsai::sim::WorldRuntime::Config runtimeConfig{};
    runtimeConfig.lap_completion_threshold = 5.0f;

    fsai::sim::MissionDefinition mission;
    runtime.Configure(mission, runtimeConfig);

    std::vector<Vector3> checkpoints = {Vector3{0.0f, 0.0f, 0.0f}, Vector3{10.0f, 0.0f, 0.0f}};
    runtime.UpdateTrackContext(checkpoints);

    Transform transform{};
    transform.position.x = 7.0f;
    transform.position.z = 0.0f;
    runtime.NotifySpawnApplied(transform);
    runtime.BeginStep(1.0);
    runtime.EvaluateLapTransition(transform);

    transform.position.x = 9.0f;  // within 5 meters of last checkpoint
    runtime.BeginStep(1.0);
    auto event = runtime.EvaluateLapTransition(transform);

    if (!event.has_value()) {
        std::cerr << "Lap transition did not trigger at configured threshold" << std::endl;
        return false;
    }

    if (event->lap_index != 1) {
        std::cerr << "Unexpected lap index after transition" << std::endl;
        return false;
    }

    return true;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= TestLoadsDefaultConfig();
    ok &= TestInvalidConfigThrows();
    ok &= TestWorldRuntimeUsesConfiguredLapMargin();
    return ok ? 0 : 1;
}

