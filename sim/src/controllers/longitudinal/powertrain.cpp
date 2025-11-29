#include "controllers/longitudinal/powertrain.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "common/errors.hpp"

namespace velox::controllers::longitudinal {

void PowertrainConfig::validate() const
{
    auto finite_and_non_negative = [](double value, const char* name) {
        if (!std::isfinite(value) || value < 0.0) {
            throw ::velox::errors::ConfigError(VELOX_LOC(std::string{name} + " must be non-negative and finite"));
        }
    };

    finite_and_non_negative(max_drive_torque, "powertrain.max_drive_torque");
    finite_and_non_negative(max_regen_torque, "powertrain.max_regen_torque");
    if (!std::isfinite(max_power) || max_power < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("powertrain.max_power must be non-negative and finite"));
    }
    finite_and_non_negative(drive_efficiency, "powertrain.drive_efficiency");
    finite_and_non_negative(regen_efficiency, "powertrain.regen_efficiency");
    if (drive_efficiency <= 0.0 || drive_efficiency > 1.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("powertrain.drive_efficiency must be in (0, 1]"));
    }
    if (regen_efficiency < 0.0 || regen_efficiency > 1.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("powertrain.regen_efficiency must be in [0, 1]"));
    }
    if (!std::isfinite(min_soc) || !std::isfinite(max_soc) || !std::isfinite(initial_soc)) {
        throw ::velox::errors::ConfigError(VELOX_LOC("powertrain SOC bounds must be finite"));
    }
    if (min_soc < 0.0 || max_soc > 1.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("powertrain SOC bounds must lie within [0, 1]"));
    }
    if (!(min_soc <= initial_soc && initial_soc <= max_soc)) {
        throw ::velox::errors::ConfigError(VELOX_LOC("0 <= min_soc <= initial_soc <= max_soc <= 1 must hold"));
    }
    if (!std::isfinite(battery_capacity_kwh) || battery_capacity_kwh <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("powertrain.battery_capacity_kwh must be positive and finite"));
    }
}

Powertrain::Powertrain(PowertrainConfig config, double wheel_radius)
    : config_(config)
    , wheel_radius_(wheel_radius)
    , capacity_joules_(config.battery_capacity_kwh * 3.6e6)
    , soc_(config.initial_soc)
{
    config_.validate();
    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel_radius must be positive and finite"));
    }
    if (!std::isfinite(capacity_joules_) || capacity_joules_ <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("battery capacity must be positive"));
    }
}

double Powertrain::available_drive_torque(double speed) const noexcept
{
    if (soc_ <= config_.min_soc) {
        return 0.0;
    }
    const double limit = torque_power_limited(speed);
    return std::clamp(limit, 0.0, config_.max_drive_torque);
}

double Powertrain::available_regen_torque(double speed) const noexcept
{
    if (soc_ >= config_.max_soc) {
        return 0.0;
    }
    if (std::abs(speed) < 1e-3) {
        return 0.0;
    }
    const double limit = torque_power_limited(speed);
    return std::clamp(limit, 0.0, config_.max_regen_torque);
}

PowertrainOutput Powertrain::step(double throttle,
                                  double regen_torque_request,
                                  double speed,
                                  double dt)
{
    throttle             = std::clamp(throttle, 0.0, 1.0);
    regen_torque_request = std::max(0.0, regen_torque_request);

    const double drive_limit  = available_drive_torque(speed);
    const double drive_torque = std::min(throttle * config_.max_drive_torque, drive_limit);
    const double regen_limit  = available_regen_torque(speed);
    const double regen_torque = std::min(regen_torque_request, regen_limit);

    const double wheel_speed = speed / wheel_radius_;
    double mechanical_drive_power = drive_torque * wheel_speed;
    double mechanical_regen_power = -regen_torque * wheel_speed;

    double battery_power = 0.0;
    if (mechanical_drive_power > 0.0) {
        battery_power += mechanical_drive_power / std::max(config_.drive_efficiency, 1e-6);
    } else {
        mechanical_drive_power = 0.0;
    }
    if (mechanical_regen_power < 0.0) {
        battery_power += mechanical_regen_power * config_.regen_efficiency;
    } else {
        mechanical_regen_power = 0.0;
    }

    const double soc_delta = (dt > 0.0) ? -battery_power * dt / capacity_joules_ : 0.0;
    soc_                   = std::clamp(soc_ + soc_delta, config_.min_soc, config_.max_soc);

    const double total_torque     = drive_torque - regen_torque;
    const double mechanical_power = mechanical_drive_power + mechanical_regen_power;

    return PowertrainOutput{total_torque, drive_torque, regen_torque, mechanical_power, battery_power};
}

void Powertrain::reset() noexcept
{
    soc_ = config_.initial_soc;
}

double Powertrain::torque_power_limited(double speed) const noexcept
{
    if (config_.max_power <= 0.0) {
        return config_.max_drive_torque;
    }
    const double wheel_speed = std::abs(speed) / wheel_radius_;
    if (wheel_speed < 1e-6) {
        return config_.max_drive_torque;
    }
    return config_.max_power / wheel_speed;
}

} // namespace velox::controllers::longitudinal
