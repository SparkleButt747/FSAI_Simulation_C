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
};

struct WorldMissionOverride {
    std::optional<std::size_t> targetLaps{};
    std::optional<bool> allowRegeneration{};
    std::optional<fsai::sim::TrackSource> trackSource{};
};

struct WorldConfig {
    fsai::sim::MissionDefinition mission;
    WorldVisibilityConfig visibility{};
    WorldTuningConfig tuning{};
    WorldControlConfig control{};
    WorldMissionOverride missionOverride{};
};

WorldConfig LoadWorldConfig(const std::string& yamlFile,
                            const fsai::sim::MissionDefinition& mission);

