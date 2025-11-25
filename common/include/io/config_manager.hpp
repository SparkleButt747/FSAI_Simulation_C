#pragma once

#include <filesystem>

#include "controllers/longitudinal/aero.hpp"
#include "controllers/longitudinal/brake.hpp"
#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/longitudinal/powertrain.hpp"
#include "controllers/longitudinal/rolling_resistance.hpp"
#include "controllers/steering_controller.hpp"
#include "simulation/low_speed_safety.hpp"
#include "simulation/loss_of_control_detector.hpp"
#include "simulation/model_timing.hpp"
#include "vehicle_parameters.hpp"
#include "yaml-cpp/yaml.h"

namespace velox::io {

class ConfigManager {
public:
    /// Construct a configuration manager rooted at the provided directories.
    /// `parameter_root` must point to the directory containing vehicle parameter YAMLs.
    /// `config_root` defaults to the sibling `config/` directory next to `parameter_root`
    /// when left empty. Both roots must exist and be directories; otherwise construction
    /// throws `errors::ConfigError`.
    explicit ConfigManager(std::filesystem::path config_root = {},
                           std::filesystem::path parameter_root = std::filesystem::path(VELOX_PARAM_ROOT));

    [[nodiscard]] const std::filesystem::path& config_root() const noexcept { return config_root_; }
    [[nodiscard]] const std::filesystem::path& parameter_root() const noexcept { return parameter_root_; }

    // Vehicle parameter loading
    [[nodiscard]] models::VehicleParameters load_vehicle_parameters(int vehicle_id) const;

    // Longitudinal subsystem
    [[nodiscard]] controllers::longitudinal::AeroConfig load_aero_config(
        const std::filesystem::path& path = "aero.yaml") const;
    [[nodiscard]] controllers::longitudinal::RollingResistanceConfig load_rolling_resistance_config(
        const std::filesystem::path& path = "rolling.yaml") const;
    [[nodiscard]] controllers::longitudinal::BrakeConfig load_brake_config(
        const std::filesystem::path& path = "brakes.yaml") const;
    [[nodiscard]] controllers::longitudinal::PowertrainConfig load_powertrain_config(
        const std::filesystem::path& path = "powertrain.yaml") const;
    [[nodiscard]] controllers::longitudinal::FinalAccelControllerConfig load_final_accel_controller_config(
        const std::filesystem::path& path = "final_accel_controller.yaml") const;

    // Steering
    [[nodiscard]] controllers::SteeringConfig load_steering_config(
        const std::filesystem::path& path = "steering.yaml") const;

    // Safety
    [[nodiscard]] simulation::LowSpeedSafetyConfig load_low_speed_safety_config(simulation::ModelType model) const;
    [[nodiscard]] simulation::LossOfControlDetectorConfig load_loss_of_control_detector_config(
        simulation::ModelType model) const;

    // Model timing
    [[nodiscard]] simulation::ModelTimingInfo load_model_timing(simulation::ModelType model) const;

private:
    std::filesystem::path config_root_;
    std::filesystem::path parameter_root_;

    [[nodiscard]] std::filesystem::path resolve_config_path(const std::filesystem::path& path) const;
    [[nodiscard]] YAML::Node load_yaml(const std::filesystem::path& path, const std::string& description) const;

    [[nodiscard]] controllers::longitudinal::AeroConfig parse_aero_config(const YAML::Node& node,
                                                                          const std::filesystem::path& path) const;
    [[nodiscard]] controllers::longitudinal::RollingResistanceConfig parse_rolling_config(
        const YAML::Node& node,
        const std::filesystem::path& path) const;
    [[nodiscard]] controllers::longitudinal::BrakeConfig parse_brake_config(const YAML::Node& node,
                                                                            const std::filesystem::path& path) const;
    [[nodiscard]] controllers::longitudinal::PowertrainConfig parse_powertrain_config(
        const YAML::Node& node,
        const std::filesystem::path& path) const;
    [[nodiscard]] controllers::longitudinal::FinalAccelControllerConfig parse_final_accel_controller_config(
        const YAML::Node& node,
        const std::filesystem::path& path) const;

    [[nodiscard]] controllers::SteeringConfig parse_steering_config(const YAML::Node& node,
                                                                    const std::filesystem::path& path) const;

    [[nodiscard]] simulation::LowSpeedSafetyConfig parse_low_speed_config(const YAML::Node& node,
                                                                          const std::filesystem::path& path) const;
    [[nodiscard]] simulation::LossOfControlDetectorConfig parse_loss_of_control_detector_config(
        const YAML::Node& node,
        const std::filesystem::path& path) const;
};

} // namespace velox::io

