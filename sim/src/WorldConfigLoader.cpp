#include "sim/WorldConfigLoader.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

using fsai::sim::MissionDefinition;
using fsai::sim::WorldConfig;
using fsai::sim::WorldRuntimeConfig;

std::string BuildMissingFieldMessage(const std::string& path, const std::string& field) {
    std::ostringstream oss;
    oss << "Missing required field '" << field << "' in " << path;
    return oss.str();
}

std::string BuildInvalidFieldMessage(const std::string& path, const std::string& field) {
    std::ostringstream oss;
    oss << "Invalid value for '" << field << "' in " << path;
    return oss.str();
}

float RequireFloat(const YAML::Node& node, const std::string& key, const std::string& path) {
    const auto value_node = node[key];
    if (!value_node || !value_node.IsScalar()) {
        throw std::runtime_error(BuildMissingFieldMessage(path, key));
    }
    try {
        return value_node.as<float>();
    } catch (const YAML::BadConversion&) {
        throw std::runtime_error(BuildInvalidFieldMessage(path, key));
    }
}

void ValidatePositive(const float value, const std::string& path, const std::string& field) {
    if (!std::isfinite(value) || value <= 0.0f) {
        throw std::runtime_error(BuildInvalidFieldMessage(path, field));
    }
}

void ValidateNonNegative(const float value, const std::string& path, const std::string& field) {
    if (!std::isfinite(value) || value < 0.0f) {
        throw std::runtime_error(BuildInvalidFieldMessage(path, field));
    }
}

}  // namespace

namespace fsai::sim {

WorldConfig WorldConfigLoader::FromFile(const std::string& path, const MissionDefinition& mission) {
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap()) {
        throw std::runtime_error("Invalid or missing YAML root: " + path);
    }

    WorldRuntimeConfig runtime{};
    runtime.collisionThreshold = RequireFloat(root, "collision_threshold", path);
    runtime.vehicleCollisionRadius = RequireFloat(root, "vehicle_collision_radius", path);
    runtime.lapCompletionThreshold = RequireFloat(root, "lap_completion_threshold", path);

    const auto controller_node = root["controller"];
    if (!controller_node || !controller_node.IsMap()) {
        throw std::runtime_error(BuildMissingFieldMessage(path, "controller"));
    }

    runtime.speedLookAheadSensitivity =
        RequireFloat(controller_node, "speed_lookahead_sensitivity", path);
    runtime.steeringLookAheadSensitivity =
        RequireFloat(controller_node, "steering_lookahead_sensitivity", path);
    runtime.accelerationFactor = RequireFloat(controller_node, "acceleration_factor", path);

    ValidatePositive(runtime.collisionThreshold, path, "collision_threshold");
    ValidatePositive(runtime.vehicleCollisionRadius, path, "vehicle_collision_radius");
    ValidatePositive(runtime.lapCompletionThreshold, path, "lap_completion_threshold");
    ValidateNonNegative(runtime.speedLookAheadSensitivity, path, "speed_lookahead_sensitivity");
    ValidateNonNegative(runtime.steeringLookAheadSensitivity, path,
                        "steering_lookahead_sensitivity");
    ValidatePositive(runtime.accelerationFactor, path, "acceleration_factor");

    WorldConfig config{mission};
    config.runtime = runtime;
    return config;
}

}  // namespace fsai::sim

