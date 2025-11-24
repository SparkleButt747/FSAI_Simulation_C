#include "WorldConfig.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
T getOrDefault(const YAML::Node& node, const char* key, T def) {
    if (auto child = node[key]) {
        return child.as<T>(def);
    }
    return def;
}

std::optional<fsai::sim::TrackSource> parseTrackSource(const YAML::Node& node) {
    if (!node) {
        return std::nullopt;
    }
    const auto value = node.as<std::string>("");
    std::string lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lowered == "random") {
        return fsai::sim::TrackSource::kRandom;
    }
    if (lowered == "csv") {
        return fsai::sim::TrackSource::kCsv;
    }
    throw std::runtime_error("Unknown track_source value: " + value);
}

}  // namespace

WorldConfig LoadWorldConfig(const std::string& yamlFile,
                            const fsai::sim::MissionDefinition& mission) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yamlFile);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load world config '" + yamlFile + "': " + e.what());
    }

    if (!root || !root.IsMap()) {
        throw std::runtime_error("World config must be a map at root: " + yamlFile);
    }

    WorldConfig config{};
    config.mission = mission;

    if (auto vis = root["visibility"]) {
        config.visibility.debug_mode = getOrDefault(vis, "debug_mode", config.visibility.debug_mode);
        config.visibility.public_ground_truth =
            getOrDefault(vis, "public_ground_truth", config.visibility.public_ground_truth);
        config.visibility.render_ground_truth =
            getOrDefault(vis, "render_ground_truth", config.visibility.render_ground_truth);
    }

    if (auto tuning = root["tuning"]) {
        config.tuning.collisionThreshold =
            getOrDefault(tuning, "collision_threshold", config.tuning.collisionThreshold);
        config.tuning.vehicleCollisionRadius =
            getOrDefault(tuning, "vehicle_collision_radius", config.tuning.vehicleCollisionRadius);
        config.tuning.lapCompletionThreshold =
            getOrDefault(tuning, "lap_completion_threshold", config.tuning.lapCompletionThreshold);
    }

    if (auto control = root["control"]) {
        config.control.useController =
            getOrDefault(control, "use_controller", config.control.useController);
        config.control.regenerateTrack =
            getOrDefault(control, "regenerate_track", config.control.regenerateTrack);
        config.control.speedLookAheadSensitivity = getOrDefault(
            control, "speed_lookahead_sensitivity", config.control.speedLookAheadSensitivity);
        config.control.steeringLookAheadSensitivity = getOrDefault(
            control, "steering_lookahead_sensitivity", config.control.steeringLookAheadSensitivity);
        config.control.accelerationFactor =
            getOrDefault(control, "acceleration_factor", config.control.accelerationFactor);
    }

    if (auto missionOverride = root["mission_override"]) {
        if (auto laps = missionOverride["target_laps"]) {
            config.missionOverride.targetLaps = laps.as<std::size_t>();
        }
        if (auto allow = missionOverride["allow_regeneration"]) {
            config.missionOverride.allowRegeneration = allow.as<bool>();
        }
        if (auto source = missionOverride["track_source"]) {
            config.missionOverride.trackSource = parseTrackSource(source);
        }
    }

    if (config.missionOverride.targetLaps) {
        config.mission.targetLaps = *config.missionOverride.targetLaps;
    }
    if (config.missionOverride.allowRegeneration) {
        config.mission.allowRegeneration = *config.missionOverride.allowRegeneration;
    }
    if (config.missionOverride.trackSource) {
        config.mission.trackSource = *config.missionOverride.trackSource;
    }

    return config;
}

