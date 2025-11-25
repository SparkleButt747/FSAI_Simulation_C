#include "WorldConfig.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
T getOrDefault(const YAML::Node& node, const char* key, T def) {
    if (auto child = node[key]) {
        return child.as<T>(def);
    }
    return def;
}

template <typename T>
T getRequired(const YAML::Node& node,
              const char* key,
              const std::string& section,
              const std::string& source) {
    auto child = node[key];
    if (!child) {
        throw std::runtime_error("Missing required key '" + section + "." + key +
                                 "' in config: " + source);
    }
    try {
        return child.as<T>();
    } catch (const std::exception& e) {
        throw std::runtime_error("Invalid value for '" + section + "." + key +
                                 "' in config: " + source + " (" + e.what() + ")");
    }
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

class WorldConfigLoader {
public:
    WorldConfigLoader(std::string source, fsai::sim::MissionDefinition mission)
        : source_(std::move(source)) {
        try {
            root_ = YAML::LoadFile(source_);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load world config '" + source_ + "': " + e.what());
        }

        if (!root_ || !root_.IsMap()) {
            throw std::runtime_error("World config must be a map at root: " + source_);
        }

        config_.mission = mission;
    }

    WorldConfig Load() {
        config_.visibility = LoadVisibility();
        config_.tuning = LoadTuning();
        config_.control = LoadControl();
        config_.missionOverride = LoadMissionOverride();
        config_.renderer = LoadRenderer();

        ApplyMissionOverride();
        Validate();

        return config_;
    }

private:
    WorldVisibilityConfig LoadVisibility() const {
        WorldVisibilityConfig visibility{};
        if (auto vis = root_["visibility"]) {
            visibility.debug_mode = getOrDefault(vis, "debug_mode", visibility.debug_mode);
            visibility.public_ground_truth =
                getOrDefault(vis, "public_ground_truth", visibility.public_ground_truth);
            visibility.render_ground_truth =
                getOrDefault(vis, "render_ground_truth", visibility.render_ground_truth);
        }
        return visibility;
    }

    WorldTuningConfig LoadTuning() const {
        const auto tuning = root_["tuning"];
        if (!tuning || !tuning.IsMap()) {
            throw std::runtime_error("Missing required 'tuning' section in config: " + source_);
        }

        WorldTuningConfig tuning_config{};
        tuning_config.collisionThreshold = getRequired<float>(tuning, "collision_threshold",
                                                              "tuning", source_);
        tuning_config.vehicleCollisionRadius = getRequired<float>(
            tuning, "vehicle_collision_radius", "tuning", source_);
        tuning_config.lapCompletionThreshold = getRequired<float>(
            tuning, "lap_completion_threshold", "tuning", source_);
        return tuning_config;
    }

    WorldControlConfig LoadControl() const {
        const auto control = root_["control"];
        if (!control || !control.IsMap()) {
            throw std::runtime_error("Missing required 'control' section in config: " + source_);
        }

        WorldControlConfig control_config{};
        control_config.useController =
            getOrDefault(control, "use_controller", control_config.useController);
        control_config.regenerateTrack =
            getOrDefault(control, "regenerate_track", control_config.regenerateTrack);
        control_config.speedLookAheadSensitivity = getRequired<float>(
            control, "speed_lookahead_sensitivity", "control", source_);
        control_config.steeringLookAheadSensitivity = getRequired<float>(
            control, "steering_lookahead_sensitivity", "control", source_);
        control_config.accelerationFactor = getRequired<float>(
            control, "acceleration_factor", "control", source_);
        control_config.pathSearchMaxLength = getRequired<std::size_t>(
            control, "path_search_max_length", "control", source_);
        control_config.pathSearchMinLength = getRequired<std::size_t>(
            control, "path_search_min_length", "control", source_);
        control_config.pathSearchBeamWidth = getRequired<std::size_t>(
            control, "path_search_beam_width", "control", source_);
        return control_config;
    }

    WorldMissionOverride LoadMissionOverride() const {
        WorldMissionOverride overrides{};
        if (auto missionOverride = root_["mission_override"]) {
            if (auto laps = missionOverride["target_laps"]) {
                overrides.targetLaps = laps.as<std::size_t>();
            }
            if (auto allow = missionOverride["allow_regeneration"]) {
                overrides.allowRegeneration = allow.as<bool>();
            }
            if (auto source = missionOverride["track_source"]) {
                overrides.trackSource = parseTrackSource(source);
            }
        }
        return overrides;
    }

    WorldRendererConfig LoadRenderer() const {
        WorldRendererConfig renderer{};
        if (auto rendererNode = root_["renderer"]) {
            renderer.enable_window =
                getOrDefault(rendererNode, "enable_window", renderer.enable_window);
            renderer.publish_stereo_frames = getOrDefault(
                rendererNode, "publish_stereo_frames", renderer.publish_stereo_frames);
            renderer.show_debug_overlays = getOrDefault(
                rendererNode, "show_debug_overlays", renderer.show_debug_overlays);
            renderer.render_scale =
                getOrDefault(rendererNode, "render_scale", renderer.render_scale);
            renderer.window_width =
                getOrDefault(rendererNode, "window_width", renderer.window_width);
            renderer.window_height =
                getOrDefault(rendererNode, "window_height", renderer.window_height);
            if (auto title = rendererNode["window_title"]) {
                renderer.window_title = title.as<std::string>(renderer.window_title);
            }
            if (auto provider = rendererNode["stereo_provider"]) {
                renderer.stereo_provider = provider.as<std::string>(renderer.stereo_provider);
            }
        }
        return renderer;
    }

    void ApplyMissionOverride() {
        if (config_.missionOverride.targetLaps) {
            config_.mission.targetLaps = *config_.missionOverride.targetLaps;
        }
        if (config_.missionOverride.allowRegeneration) {
            config_.mission.allowRegeneration = *config_.missionOverride.allowRegeneration;
        }
        if (config_.missionOverride.trackSource) {
            config_.mission.trackSource = *config_.missionOverride.trackSource;
        }
    }

    void Validate() const {
        const auto& tuning = config_.tuning;
        if (tuning.collisionThreshold <= 0.0f) {
            throw std::runtime_error("tuning.collision_threshold must be > 0: " + source_);
        }
        if (tuning.vehicleCollisionRadius <= 0.0f) {
            throw std::runtime_error("tuning.vehicle_collision_radius must be > 0: " + source_);
        }
        if (tuning.lapCompletionThreshold <= 0.0f) {
            throw std::runtime_error("tuning.lap_completion_threshold must be > 0: " + source_);
        }

        const auto& control = config_.control;
        if (control.speedLookAheadSensitivity < 0.0f) {
            throw std::runtime_error("control.speed_lookahead_sensitivity must be >= 0: " + source_);
        }
        if (control.steeringLookAheadSensitivity < 0.0f) {
            throw std::runtime_error("control.steering_lookahead_sensitivity must be >= 0: " + source_);
        }
        if (control.accelerationFactor <= 0.0f) {
            throw std::runtime_error("control.acceleration_factor must be > 0: " + source_);
        }
        if (control.pathSearchMaxLength == 0) {
            throw std::runtime_error("control.path_search_max_length must be > 0: " + source_);
        }
        if (control.pathSearchMinLength == 0 ||
            control.pathSearchMinLength > control.pathSearchMaxLength) {
            throw std::runtime_error(
                "control.path_search_min_length must be > 0 and <= path_search_max_length: " + source_);
        }
        if (control.pathSearchBeamWidth == 0) {
            throw std::runtime_error("control.path_search_beam_width must be > 0: " + source_);
        }

        if (config_.renderer.render_scale <= 0.0f) {
            throw std::runtime_error("renderer.render_scale must be > 0: " + source_);
        }
    }

    YAML::Node root_{};
    WorldConfig config_{};
    std::string source_{};
};

}  // namespace

WorldConfig LoadWorldConfig(const std::string& yamlFile,
                            const fsai::sim::MissionDefinition& mission) {
    return WorldConfigLoader(yamlFile, mission).Load();
}
