#include "controllers/steering_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "common/errors.hpp"

namespace velox::controllers {

using velox::models::utils::SteeringParameters;

namespace {
constexpr double EPS = 1e-9;

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

double clamp01(double value)
{
    return clamp(value, 0.0, 1.0);
}

double combined_min_angle(const SteeringConfig::Final& cfg, const SteeringParameters& limits)
{
    if (limits.max <= limits.min) {
        return cfg.min_angle;
    }
    return std::max(cfg.min_angle, limits.min);
}

double combined_max_angle(const SteeringConfig::Final& cfg, const SteeringParameters& limits)
{
    if (limits.max <= limits.min) {
        return cfg.max_angle;
    }
    return std::min(cfg.max_angle, limits.max);
}

double combined_rate_min(const SteeringConfig::Final& cfg, const SteeringParameters& limits)
{
    double rate_min = -cfg.max_rate;
    if (limits.v_min < 0.0) {
        rate_min = std::max(rate_min, limits.v_min);
    }
    return rate_min;
}

double combined_rate_max(const SteeringConfig::Final& cfg, const SteeringParameters& limits)
{
    double rate_max = cfg.max_rate;
    if (limits.v_max > 0.0) {
        rate_max = std::min(rate_max, limits.v_max);
    }
    return rate_max;
}

} // namespace

void SteeringConfig::Wheel::validate() const
{
    if (max_angle <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel.max_angle must be positive"));
    }
    if (max_rate <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel.max_rate must be positive"));
    }
    if (nudge_angle <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel.nudge_angle must be positive"));
    }
    if (centering_stiffness <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel.centering_stiffness must be positive"));
    }
    if (centering_deadband < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel.centering_deadband cannot be negative"));
    }
    if (centering_deadband >= max_angle) {
        throw ::velox::errors::ConfigError(VELOX_LOC("wheel.centering_deadband must be smaller than wheel.max_angle"));
    }
}

void SteeringConfig::Final::validate() const
{
    if (max_angle <= min_angle) {
        throw ::velox::errors::ConfigError(VELOX_LOC("final.max_angle must be greater than final.min_angle"));
    }
    if (max_rate <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("final.max_rate must be positive"));
    }
    if (actuator_time_constant <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("final.actuator_time_constant must be positive"));
    }
    if (smoothing_time_constant < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("final.smoothing_time_constant cannot be negative"));
    }
}

void SteeringConfig::validate() const
{
    wheel.validate();
    final.validate();
    if (final.min_angle > -wheel.max_angle || final.max_angle < wheel.max_angle) {
        throw ::velox::errors::ConfigError(VELOX_LOC("final angle range must encompass wheel range"));
    }
}

SteeringWheel::SteeringWheel(const SteeringConfig::Wheel& cfg,
                             const SteeringParameters& limits)
    : cfg_(cfg)
    , limits_(limits)
{
    reset(0.0);
}

double SteeringWheel::max_left() const
{
    double limit = cfg_.max_angle;
    if (limits_.max > 0.0) {
        limit = std::min(limit, limits_.max);
    }
    return limit;
}

double SteeringWheel::max_right() const
{
    double limit = -cfg_.max_angle;
    if (limits_.min < 0.0) {
        limit = std::max(limit, limits_.min);
    }
    return limit;
}

void SteeringWheel::reset(double angle)
{
    angle_                   = clamp(angle, max_right(), max_left());
    last_output_.angle       = angle_;
    last_output_.rate        = 0.0;
    last_output_.target_angle = angle_;
}

SteeringWheel::Output SteeringWheel::update(double nudge, double dt)
{
    if (dt <= 0.0) {
        throw ::velox::errors::InputError(VELOX_LOC("dt must be positive"));
    }

    const double prev_angle = angle_;
    const double left_limit = max_left();
    const double right_limit = max_right();

    double clamped_nudge = std::clamp(nudge, -1.0, 1.0);
    double angle = angle_;

    if (std::abs(clamped_nudge) > EPS) {
        double target = angle + clamped_nudge * cfg_.nudge_angle;
        target        = clamp(target, right_limit, left_limit);
        const double max_delta = cfg_.max_rate * dt;
        const double delta     = clamp(target - angle, -max_delta, max_delta);
        angle += delta;
    } else {
        if (std::abs(angle) <= cfg_.centering_deadband) {
            angle = 0.0;
        } else {
            const double effective = std::copysign(
                std::max(0.0, std::abs(angle) - cfg_.centering_deadband), angle);
            double rate = -cfg_.centering_stiffness * effective;
            rate        = clamp(rate, -cfg_.max_rate, cfg_.max_rate);
            angle += rate * dt;
            if (angle == 0.0 || std::copysign(1.0, angle) != std::copysign(1.0, angle_)) {
                angle = 0.0;
            }
        }
    }

    angle_ = clamp(angle, right_limit, left_limit);

    last_output_.target_angle = angle_;
    last_output_.angle        = angle_;
    last_output_.rate         = (angle_ - prev_angle) / dt;
    return last_output_;
}

FinalSteerController::FinalSteerController(const SteeringConfig::Final& cfg,
                                           const SteeringParameters& limits)
    : cfg_(cfg)
    , limits_(limits)
{
    reset(0.0);
}

void FinalSteerController::reset(double current_angle)
{
    const double clamped = clamp(current_angle, min_angle(), max_angle());
    filtered_target_           = clamped;
    last_output_.filtered_target = clamped;
    last_output_.angle          = clamped;
    last_output_.rate           = 0.0;
}

double FinalSteerController::min_angle() const
{
    return combined_min_angle(cfg_, limits_);
}

double FinalSteerController::max_angle() const
{
    return combined_max_angle(cfg_, limits_);
}

FinalSteerController::Output FinalSteerController::update(double desired_angle,
                                                          double current_angle,
                                                          double dt)
{
    if (dt <= 0.0) {
        throw ::velox::errors::InputError(VELOX_LOC("dt must be positive"));
    }

    const double min_angle_value = min_angle();
    const double max_angle_value = max_angle();

    const double desired_clamped  = clamp(desired_angle, min_angle_value, max_angle_value);
    const double measured_clamped = clamp(current_angle, min_angle_value, max_angle_value);

    if (cfg_.smoothing_time_constant > 0.0) {
        const double alpha = clamp01(dt / (cfg_.smoothing_time_constant + dt));
        filtered_target_ += alpha * (desired_clamped - filtered_target_);
    } else {
        filtered_target_ = desired_clamped;
    }
    filtered_target_ = clamp(filtered_target_, min_angle_value, max_angle_value);

    const double tau  = cfg_.actuator_time_constant;
    double       rate = (filtered_target_ - measured_clamped) / tau;

    const double rate_min = combined_rate_min(cfg_, limits_);
    const double rate_max = combined_rate_max(cfg_, limits_);
    rate                   = clamp(rate, rate_min, rate_max);

    const double max_step = (max_angle_value - measured_clamped) / dt;
    const double min_step = (min_angle_value - measured_clamped) / dt;
    rate                  = clamp(rate, min_step, max_step);

    double new_angle = measured_clamped + rate * dt;
    new_angle        = clamp(new_angle, min_angle_value, max_angle_value);

    rate = (new_angle - measured_clamped) / dt;

    last_output_.filtered_target = filtered_target_;
    last_output_.angle           = new_angle;
    last_output_.rate            = rate;
    return last_output_;
}

FinalSteerController::Output FinalSteerController::update_absolute(double desired_angle,
                                                                  double current_angle,
                                                                  double dt)
{
    return update(desired_angle, current_angle, dt);
}

} // namespace velox::controllers
