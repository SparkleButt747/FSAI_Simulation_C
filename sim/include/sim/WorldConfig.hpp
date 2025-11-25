#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "sim/cone_constants.hpp"
#include "sim/mission/MissionDefinition.hpp"

struct WorldVisibilityConfig {
    bool debug_mode{true};
    bool public_ground_truth{true};
    bool render_ground_truth{true};
};

struct WorldTuningConfig {
    float collisionThreshold{1.75f};
    float vehicleCollisionRadius{0.5f - fsai::sim::kSmallConeRadiusMeters};
    float lapCompletionThreshold{0.2f};
};

struct WorldControlConfig {
    bool useController{true};
    bool regenerateTrack{true};
    float speedLookAheadSensitivity{0.5f};
    float steeringLookAheadSensitivity{0.0f};
    float accelerationFactor{0.0019f};
    std::size_t pathSearchMaxLength{30};
    std::size_t pathSearchMinLength{2};
    std::size_t pathSearchBeamWidth{20};
};

struct WorldMissionOverride {
    std::optional<std::size_t> targetLaps{};
    std::optional<bool> allowRegeneration{};
    std::optional<fsai::sim::TrackSource> trackSource{};
};

struct WorldRendererConfig {
    bool enable_window{true};
    bool publish_stereo_frames{true};
    bool show_debug_overlays{true};
    float render_scale{5.0f};
    int window_width{800};
    int window_height{600};
    std::string window_title{"Car Simulation 2D"};
    std::string stereo_provider{"sim_stereo"};
};

struct WorldConfig {
    fsai::sim::MissionDefinition mission;
    WorldVisibilityConfig visibility{};
    WorldTuningConfig tuning{};
    WorldControlConfig control{};
    WorldMissionOverride missionOverride{};
    WorldRendererConfig renderer{};
};

WorldConfig LoadWorldConfig(const std::string& yamlFile,
                            const fsai::sim::MissionDefinition& mission);
