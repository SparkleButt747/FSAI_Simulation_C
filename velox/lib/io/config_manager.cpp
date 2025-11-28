#include "io/config_manager.hpp"

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>

#include "common/errors.hpp"

namespace velox::io {

namespace {
namespace fs = std::filesystem;

template <typename T>
T required_scalar(const YAML::Node& node, const std::string& key, const fs::path& path)
{
    auto value = node[key];
    if (!value) {
        std::ostringstream oss;
        oss << "Missing required key '" << key << "' in " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    try {
        return value.as<T>();
    } catch (const YAML::BadConversion& ex) {
        std::ostringstream oss;
        oss << "Invalid type for key '" << key << "' in " << path.string() << ": " << ex.what();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
}

std::string model_key(simulation::ModelType model)
{
    switch (model) {
        case simulation::ModelType::MB: return "mb";
        case simulation::ModelType::ST: return "st";
        case simulation::ModelType::STD: return "std";
    }
    std::ostringstream oss;
    oss << "Unsupported model type: " << static_cast<int>(model);
    throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
}

} // namespace

ConfigManager::ConfigManager(std::filesystem::path config_root, std::filesystem::path parameter_root)
    : parameter_root_(std::move(parameter_root))
{
    if (!fs::exists(parameter_root_) || !fs::is_directory(parameter_root_)) {
        std::ostringstream oss;
        oss << "Parameter root is not a valid directory: " << parameter_root_.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }

    if (config_root.empty()) {
        config_root_ = parameter_root_.parent_path() / "config";
    } else {
        config_root_ = std::move(config_root);
    }

    if (!fs::exists(config_root_) || !fs::is_directory(config_root_)) {
        std::ostringstream oss;
        oss << "Config root is not a valid directory: " << config_root_.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
}

models::VehicleParameters ConfigManager::load_vehicle_parameters(int vehicle_id) const
{
    return models::setup_vehicle_parameters(vehicle_id, parameter_root_.string());
}

controllers::longitudinal::AeroConfig ConfigManager::load_aero_config(const std::filesystem::path& path) const
{
    const auto resolved = resolve_config_path(path);
    const auto node     = load_yaml(resolved, "aero");
    return parse_aero_config(node, resolved);
}

controllers::longitudinal::RollingResistanceConfig
ConfigManager::load_rolling_resistance_config(const std::filesystem::path& path) const
{
    const auto resolved = resolve_config_path(path);
    const auto node     = load_yaml(resolved, "rolling resistance");
    return parse_rolling_config(node, resolved);
}

controllers::longitudinal::BrakeConfig ConfigManager::load_brake_config(const std::filesystem::path& path) const
{
    const auto resolved = resolve_config_path(path);
    const auto node     = load_yaml(resolved, "brake");
    return parse_brake_config(node, resolved);
}

controllers::longitudinal::PowertrainConfig
ConfigManager::load_powertrain_config(const std::filesystem::path& path) const
{
    const auto resolved = resolve_config_path(path);
    const auto node     = load_yaml(resolved, "powertrain");
    return parse_powertrain_config(node, resolved);
}

controllers::longitudinal::FinalAccelControllerConfig
ConfigManager::load_final_accel_controller_config(const std::filesystem::path& path) const
{
    const auto resolved = resolve_config_path(path);
    const auto node     = load_yaml(resolved, "final accel controller");
    return parse_final_accel_controller_config(node, resolved);
}

controllers::SteeringConfig ConfigManager::load_steering_config(const std::filesystem::path& path) const
{
    const auto resolved = resolve_config_path(path);
    const auto node     = load_yaml(resolved, "steering");
    return parse_steering_config(node, resolved);
}

simulation::LowSpeedSafetyConfig ConfigManager::load_low_speed_safety_config(simulation::ModelType model) const
{
    const auto suffix         = model_key(model);
    const auto override_name  = fs::path("low_speed_safety_" + suffix + ".yaml");
    const auto override_path  = resolve_config_path(override_name);
    const auto default_path   = resolve_config_path("low_speed_safety.yaml");
    const bool override_exists = fs::exists(override_path);

    const fs::path& path = override_exists ? override_path : default_path;
    const auto node      = load_yaml(path, "low speed safety");
    return parse_low_speed_config(node, path);
}

simulation::LossOfControlDetectorConfig
ConfigManager::load_loss_of_control_detector_config(simulation::ModelType model) const
{
    const auto key  = model_key(model);
    const auto path = resolve_config_path("loss_of_control_detector.yaml");
    const auto node = load_yaml(path, "loss of control detector");
    const auto root = node[key];
    if (!root || !root.IsMap()) {
        std::ostringstream oss;
        oss << "loss_of_control_detector.yaml missing section for '" << key << "'";
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    return parse_loss_of_control_detector_config(root, path);
}

simulation::ModelTimingInfo ConfigManager::load_model_timing(simulation::ModelType model) const
{
    const auto default_timing = simulation::model_timing(model);
    const auto timing_path    = resolve_config_path("model_timing.yaml");
    if (!std::filesystem::exists(timing_path)) {
        return default_timing;
    }

    const auto key  = model_key(model);
    const auto node = load_yaml(timing_path, "model timing");
    const auto root = node[key];
    if (!root || !root.IsMap()) {
        return default_timing;
    }

    simulation::ModelTimingInfo info{};
    info.nominal_dt = required_scalar<float>(root, "nominal_dt", timing_path);
    info.max_dt     = required_scalar<float>(root, "max_dt", timing_path);

    if (info.nominal_dt <= 0.0f) {
        std::ostringstream oss;
        oss << "nominal_dt for '" << key << "' in " << timing_path.string() << " must be positive";
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    if (info.max_dt < info.nominal_dt || info.max_dt < simulation::kMinStableDt) {
        std::ostringstream oss;
        oss << "max_dt for '" << key << "' in " << timing_path.string()
            << " must be >= nominal_dt and >= " << simulation::kMinStableDt;
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }

    return info;
}

std::filesystem::path ConfigManager::resolve_config_path(const std::filesystem::path& path) const
{
    if (path.is_absolute()) {
        return path;
    }
    return config_root_ / path;
}

YAML::Node ConfigManager::load_yaml(const std::filesystem::path& path, const std::string& description) const
{
    if (!std::filesystem::exists(path)) {
        std::ostringstream oss;
        oss << "Missing " << description << " config file: " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }

    try {
        return YAML::LoadFile(path.string());
    } catch (const YAML::ParserException& ex) {
        std::ostringstream oss;
        oss << "Failed to parse " << description << " config at " << path.string() << ": " << ex.what();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
}

controllers::longitudinal::AeroConfig ConfigManager::parse_aero_config(const YAML::Node& node,
                                                                      const std::filesystem::path& path) const
{
    controllers::longitudinal::AeroConfig cfg{};
    cfg.drag_coefficient      = required_scalar<double>(node, "drag_coefficient", path);
    cfg.downforce_coefficient = required_scalar<double>(node, "downforce_coefficient", path);
    cfg.validate();
    return cfg;
}

controllers::longitudinal::RollingResistanceConfig
ConfigManager::parse_rolling_config(const YAML::Node& node, const std::filesystem::path& path) const
{
    controllers::longitudinal::RollingResistanceConfig cfg{};
    cfg.c_rr = required_scalar<double>(node, "c_rr", path);
    cfg.validate();
    return cfg;
}

controllers::longitudinal::BrakeConfig ConfigManager::parse_brake_config(const YAML::Node& node,
                                                                          const std::filesystem::path& path) const
{
    controllers::longitudinal::BrakeConfig cfg{};
    cfg.max_force       = required_scalar<double>(node, "max_force", path);
    cfg.max_regen_force = required_scalar<double>(node, "max_regen_force", path);
    cfg.min_regen_speed = required_scalar<double>(node, "min_regen_speed", path);
    cfg.validate();
    return cfg;
}

controllers::longitudinal::PowertrainConfig
ConfigManager::parse_powertrain_config(const YAML::Node& node, const std::filesystem::path& path) const
{
    controllers::longitudinal::PowertrainConfig cfg{};
    cfg.max_drive_torque     = required_scalar<double>(node, "max_drive_torque", path);
    cfg.max_regen_torque     = required_scalar<double>(node, "max_regen_torque", path);
    cfg.max_power            = required_scalar<double>(node, "max_power", path);
    cfg.drive_efficiency     = required_scalar<double>(node, "drive_efficiency", path);
    cfg.regen_efficiency     = required_scalar<double>(node, "regen_efficiency", path);
    cfg.min_soc              = required_scalar<double>(node, "min_soc", path);
    cfg.max_soc              = required_scalar<double>(node, "max_soc", path);
    cfg.initial_soc          = required_scalar<double>(node, "initial_soc", path);
    cfg.battery_capacity_kwh = required_scalar<double>(node, "battery_capacity_kwh", path);
    cfg.validate();
    return cfg;
}

controllers::longitudinal::FinalAccelControllerConfig ConfigManager::parse_final_accel_controller_config(
    const YAML::Node& node,
    const std::filesystem::path& path) const
{
    controllers::longitudinal::FinalAccelControllerConfig cfg{};
    cfg.tau_throttle = required_scalar<double>(node, "tau_throttle", path);
    cfg.tau_brake    = required_scalar<double>(node, "tau_brake", path);
    cfg.accel_min    = required_scalar<double>(node, "accel_min", path);
    cfg.accel_max    = required_scalar<double>(node, "accel_max", path);
    if (auto stop = node["stop_speed_epsilon"]) {
        cfg.stop_speed_epsilon = stop.as<double>();
    }
    cfg.validate();
    return cfg;
}

controllers::SteeringConfig ConfigManager::parse_steering_config(const YAML::Node& node,
                                                                  const std::filesystem::path& path) const
{
    controllers::SteeringConfig cfg{};

    const auto wheel = node["wheel"];
    if (!wheel || !wheel.IsMap()) {
        std::ostringstream oss;
        oss << "steering config missing 'wheel' section in " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    cfg.wheel.max_angle           = required_scalar<double>(wheel, "max_angle", path);
    cfg.wheel.max_rate            = required_scalar<double>(wheel, "max_rate", path);
    cfg.wheel.nudge_angle         = required_scalar<double>(wheel, "nudge_angle", path);
    cfg.wheel.centering_stiffness = required_scalar<double>(wheel, "centering_stiffness", path);
    cfg.wheel.centering_deadband  = required_scalar<double>(wheel, "centering_deadband", path);

    const auto final = node["final"];
    if (!final || !final.IsMap()) {
        std::ostringstream oss;
        oss << "steering config missing 'final' section in " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    cfg.final.min_angle               = required_scalar<double>(final, "min_angle", path);
    cfg.final.max_angle               = required_scalar<double>(final, "max_angle", path);
    cfg.final.max_rate                = required_scalar<double>(final, "max_rate", path);
    cfg.final.actuator_time_constant  = required_scalar<double>(final, "actuator_time_constant", path);
    cfg.final.smoothing_time_constant = required_scalar<double>(final, "smoothing_time_constant", path);

    cfg.validate();
    return cfg;
}

simulation::LowSpeedSafetyConfig ConfigManager::parse_low_speed_config(const YAML::Node& node,
                                                                      const std::filesystem::path& path) const
{
    simulation::LowSpeedSafetyConfig cfg{};
    const auto normal = node["normal"];
    const auto drift  = node["drift"];
    if (!normal || !normal.IsMap()) {
        std::ostringstream oss;
        oss << "low speed safety config missing 'normal' section in " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    if (!drift || !drift.IsMap()) {
        std::ostringstream oss;
        oss << "low speed safety config missing 'drift' section in " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }

    const auto parse_profile = [&](const YAML::Node& profile, const char* name) {
        simulation::LowSpeedSafetyProfile cfg_profile{};
        cfg_profile.engage_speed     = required_scalar<double>(profile, "engage_speed", path);
        cfg_profile.release_speed    = required_scalar<double>(profile, "release_speed", path);
        cfg_profile.yaw_rate_limit   = required_scalar<double>(profile, "yaw_rate_limit", path);
        cfg_profile.slip_angle_limit = required_scalar<double>(profile, "slip_angle_limit", path);
        try {
            cfg_profile.validate(name);
        } catch (const std::exception&) {
            throw;
        }
        return cfg_profile;
    };

    cfg.normal             = parse_profile(normal, "normal");
    cfg.drift              = parse_profile(drift, "drift");
    cfg.stop_speed_epsilon = required_scalar<double>(node, "stop_speed_epsilon", path);
    const auto drift_enabled_node = node["drift_enabled"];
    if (!drift_enabled_node) {
        std::ostringstream oss;
        oss << "low speed safety config missing 'drift_enabled' in " << path.string();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    try {
        cfg.drift_enabled = drift_enabled_node.as<bool>();
    } catch (const YAML::BadConversion& ex) {
        std::ostringstream oss;
        oss << "Invalid type for key 'drift_enabled' in " << path.string() << ": " << ex.what();
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    cfg.validate();
    return cfg;
}

simulation::LossOfControlDetectorConfig ConfigManager::parse_loss_of_control_detector_config(
    const YAML::Node& node,
    const std::filesystem::path& path) const
{
    const auto parse_metric = [&](const char* key) {
        const auto metric = node[key];
        if (!metric || !metric.IsMap()) {
            std::ostringstream oss;
            oss << "loss of control detector config missing '" << key << "' section in " << path.string();
            throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
        }

        simulation::MetricThreshold thresholds{};
        thresholds.magnitude_threshold = required_scalar<double>(metric, "threshold", path);
        thresholds.rate_threshold       = required_scalar<double>(metric, "rate", path);
        return thresholds;
    };

    simulation::LossOfControlDetectorConfig cfg{};
    cfg.yaw_rate      = parse_metric("yaw_rate");
    cfg.slip_angle    = parse_metric("slip_angle");
    cfg.lateral_accel = parse_metric("lateral_accel");
    cfg.slip_ratio    = parse_metric("slip_ratio");
    cfg.validate();
    return cfg;
}

} // namespace velox::io

