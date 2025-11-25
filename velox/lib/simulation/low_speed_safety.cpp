#include "simulation/low_speed_safety.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "common/errors.hpp"
namespace velox::simulation {

void LowSpeedSafetyProfile::validate(const char* name) const
{
    const auto error = [name](std::string_view field, std::string_view msg) {
        std::ostringstream oss;
        oss << name << "." << field << " " << msg;
        return ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    };

    if (engage_speed < 0.0) {
        throw error("engage_speed", "must be non-negative");
    }
    if (release_speed <= 0.0) {
        throw error("release_speed", "must be positive");
    }
    if (release_speed < engage_speed) {
        throw error("release_speed", "must be >= engage_speed");
    }
    if (yaw_rate_limit <= 0.0) {
        throw error("yaw_rate_limit", "must be positive");
    }
    if (slip_angle_limit <= 0.0) {
        throw error("slip_angle_limit", "must be positive");
    }
}

void LowSpeedSafetyConfig::validate() const
{
    normal.validate("normal");
    drift.validate("drift");
    if (stop_speed_epsilon < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("stop_speed_epsilon must be non-negative"));
    }
}

LowSpeedSafety::LowSpeedSafety(const LowSpeedSafetyConfig& config,
                               std::optional<int> longitudinal_index,
                               std::optional<int> lateral_index,
                               std::optional<int> yaw_rate_index,
                               std::optional<int> slip_index,
                               std::vector<int> wheel_speed_indices,
                               std::optional<int> steering_index,
                               std::optional<double> wheelbase,
                               std::optional<double> rear_length)
    : config_(config)
    , longitudinal_index_(longitudinal_index)
    , lateral_index_(lateral_index)
    , yaw_rate_index_(yaw_rate_index)
    , slip_index_(slip_index)
    , wheel_speed_indices_(std::move(wheel_speed_indices))
    , steering_index_(steering_index)
    , wheelbase_(wheelbase)
    , rear_length_(rear_length)
{
    config_.validate();
    drift_enabled_ = config_.drift_enabled;
}

void LowSpeedSafety::reset()
{
    engaged_ = false;
}

namespace {
} // namespace

struct LowSpeedSafety::MonitoredMetrics
{
    double                speed            = 0.0;
    double                severity         = 0.0;
    double                transition_blend = 0.0;
    bool                  drift_mode       = false;
    bool                  near_stop        = false;
    std::optional<double> yaw_state{};
    std::optional<double> slip_state{};
    std::optional<double> yaw_target{};
    std::optional<double> slip_target{};
    std::optional<double> lateral_target{};
    std::optional<double> velocity_heading{};
};

struct LowSpeedSafety::SafetyDecision
{
    SafetyStage mode            = SafetyStage::Normal;
    bool       latch_active     = false;
    double     transition_blend = 0.0;
    bool       severity_trip    = false;
};

LowSpeedSafety::MonitoredMetrics LowSpeedSafety::monitor(const std::vector<double>& state,
                                                         double                     speed,
                                                         const LowSpeedSafetyProfile& profile) const
{
    MonitoredMetrics metrics{};
    metrics.speed            = speed;
    metrics.drift_mode       = drift_enabled_;
    metrics.near_stop        = std::abs(speed) <= config_.stop_speed_epsilon;
    metrics.transition_blend = pre_latch_blend(speed, profile);

    if (yaw_rate_index_ && index_in_bounds(*yaw_rate_index_, state)) {
        metrics.yaw_state = state[static_cast<std::size_t>(*yaw_rate_index_)];
    }
    if (slip_index_ && index_in_bounds(*slip_index_, state)) {
        metrics.slip_state = state[static_cast<std::size_t>(*slip_index_)];
    }

    metrics.yaw_target       = kinematic_yaw_rate(state, speed);
    metrics.slip_target      = kinematic_slip(state, speed);
    metrics.lateral_target   = kinematic_lateral_velocity(state, speed);
    metrics.velocity_heading = velocity_slip(state);

    const double yaw_limit  = std::max(profile.yaw_rate_limit, 1e-6);
    const double slip_limit = std::max(profile.slip_angle_limit, 1e-6);
    double       yaw_ratio  = 0.0;
    double       slip_ratio = 0.0;

    if (metrics.yaw_state.has_value()) {
        yaw_ratio = std::abs(*metrics.yaw_state) / yaw_limit;
    }
    if (metrics.slip_state.has_value()) {
        slip_ratio = std::abs(*metrics.slip_state) / slip_limit;
    }
    metrics.severity = std::max(yaw_ratio, slip_ratio);

    return metrics;
}

LowSpeedSafety::SafetyDecision LowSpeedSafety::decide(const LowSpeedSafety::MonitoredMetrics& metrics,
                                                      const LowSpeedSafetyProfile& profile,
                                                      bool update_latch)
{
    const bool severity_trip = metrics.severity > 1.0;

    if (update_latch) {
        if (engaged_) {
            if (metrics.speed > profile.release_speed && !severity_trip) {
                engaged_ = false;
            }
        } else {
            if (metrics.speed < profile.engage_speed || severity_trip) {
                engaged_ = true;
            }
        }
    }

    SafetyDecision decision{};
    decision.severity_trip    = severity_trip;
    decision.transition_blend = metrics.transition_blend;
    decision.latch_active     = engaged_ || severity_trip;
    if (decision.latch_active) {
        decision.mode = SafetyStage::Emergency;
    } else if (metrics.transition_blend > 0.0) {
        decision.mode = SafetyStage::Transition;
    } else {
        decision.mode = SafetyStage::Normal;
    }

    return decision;
}

void LowSpeedSafety::clamp_state(std::vector<double>&              state,
                                 const LowSpeedSafety::MonitoredMetrics& metrics,
                                 const LowSpeedSafety::SafetyDecision& decision,
                                 const LowSpeedSafetyProfile& profile)
{
    const bool drift_mode       = metrics.drift_mode;
    const bool allow_unclamped  = drift_mode && decision.mode == SafetyStage::Normal && decision.transition_blend <= 0.0;
    const bool wheel_stage_latch = decision.latch_active || metrics.speed < profile.engage_speed || decision.transition_blend > 0.0;

    auto yaw_target       = metrics.yaw_target;
    auto slip_target      = metrics.slip_target;
    auto lateral_target   = metrics.lateral_target;
    auto velocity_heading = metrics.velocity_heading;

    if (decision.mode == SafetyStage::Emergency) {
        const double beta_ref     = velocity_heading.value_or(0.0);
        const double slip_command = (velocity_heading.has_value() && !metrics.near_stop) ? beta_ref : 0.0;

        yaw_target     = 0.0;
        slip_target    = slip_command;
        if (velocity_heading.has_value() && !metrics.near_stop) {
            lateral_target = metrics.speed * std::sin(slip_command);
        } else {
            lateral_target = 0.0;
        }
    }

    if (yaw_rate_index_ && index_in_bounds(*yaw_rate_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*yaw_rate_index_);
        if (decision.mode == SafetyStage::Emergency) {
            const double limit  = profile.yaw_rate_limit;
            const double target = yaw_target.value_or(0.0);
            state[idx]          = clamp(target, -limit, limit);
        } else if (!allow_unclamped) {
            double limit = scaled_limit(profile.yaw_rate_limit, metrics.speed, profile);
            double value = clamp(state[idx], -limit, limit);
            if (decision.transition_blend > 0.0 && yaw_target.has_value()) {
                const double target = clamp(*yaw_target, -limit, limit);
                value = (1.0 - decision.transition_blend) * value + decision.transition_blend * target;
            }
            state[idx] = value;
        }
    }

    if (lateral_index_ && index_in_bounds(*lateral_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*lateral_index_);
        double value = state[idx];
        if (decision.mode == SafetyStage::Emergency) {
            if (lateral_target.has_value()) {
                state[idx] = *lateral_target;
            } else {
                const double limit = config_.stop_speed_epsilon;
                state[idx] = clamp(value, -limit, limit);
            }
        } else if (std::fabs(value) <= config_.stop_speed_epsilon) {
            state[idx] = 0.0;
        }
    }

    if (slip_index_ && index_in_bounds(*slip_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*slip_index_);
        if (decision.mode == SafetyStage::Emergency) {
            const double limit  = profile.slip_angle_limit;
            const double target = slip_target.value_or(0.0);
            state[idx]          = clamp(target, -limit, limit);
        } else if (!allow_unclamped) {
            double limit = scaled_limit(profile.slip_angle_limit, metrics.speed, profile);
            double value = clamp(state[idx], -limit, limit);
            double target = 0.0;
            if (velocity_heading.has_value() && !metrics.near_stop) {
                target = clamp(*velocity_heading, -limit, limit);
            }
            if (decision.transition_blend > 0.0) {
                value = (1.0 - decision.transition_blend) * value + decision.transition_blend * target;
            }
            state[idx] = value;
        }
    }

    if (!wheel_speed_indices_.empty()) {
        for (int raw_idx : wheel_speed_indices_) {
            if (!index_in_bounds(raw_idx, state)) {
                continue;
            }
            const std::size_t idx = static_cast<std::size_t>(raw_idx);
            double value = state[idx];
            if (value <= 0.0) {
                state[idx] = 0.0;
            } else if (wheel_stage_latch && value <= config_.stop_speed_epsilon) {
                state[idx] = 0.0;
            }
        }
    }
}

void LowSpeedSafety::apply(std::vector<double>& state, double speed, bool update_latch)
{
    const auto& profile  = config_.active_profile(drift_enabled_);
    const auto  metrics  = monitor(state, speed, profile);
    const auto  decision = decide(metrics, profile, update_latch);
    clamp_state(state, metrics, decision, profile);
}

LowSpeedSafetyStatus LowSpeedSafety::status(const std::vector<double>& state, double speed) const
{
    const auto& profile = config_.active_profile(drift_enabled_);
    const auto  metrics = monitor(state, speed, profile);

    LowSpeedSafetyStatus status{};
    status.severity         = metrics.severity;
    status.transition_blend = metrics.transition_blend;
    status.drift_mode       = metrics.drift_mode;
    status.detector_forced  = metrics.severity > 1.0;
    status.latch_active     = engaged_ || status.detector_forced;
    if (status.latch_active) {
        status.stage = SafetyStage::Emergency;
    } else if (metrics.transition_blend > 0.0) {
        status.stage = SafetyStage::Transition;
    }

    return status;
}

bool LowSpeedSafety::index_in_bounds(int index, const std::vector<double>& state)
{
    return index >= 0 && static_cast<std::size_t>(index) < state.size();
}

double LowSpeedSafety::clamp(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(max_value, value));
}

std::optional<double> LowSpeedSafety::kinematic_beta(const std::vector<double>& state) const
{
    if (!steering_index_.has_value() || !wheelbase_.has_value() || !rear_length_.has_value()) {
        return std::nullopt;
    }
    if (!index_in_bounds(*steering_index_, state)) {
        return std::nullopt;
    }
    if (*wheelbase_ <= 0.0) {
        return std::nullopt;
    }
    const double delta = state[static_cast<std::size_t>(*steering_index_)];
    const double ratio = *rear_length_ / *wheelbase_;
    return std::atan(std::tan(delta) * ratio);
}

std::optional<double> LowSpeedSafety::kinematic_yaw_rate(const std::vector<double>& state, double speed) const
{
    const auto beta = kinematic_beta(state);
    if (!beta.has_value() || !steering_index_.has_value() || !wheelbase_.has_value()) {
        return std::nullopt;
    }
    if (!index_in_bounds(*steering_index_, state)) {
        return std::nullopt;
    }
    if (std::fabs(speed) <= 1e-9) {
        return 0.0;
    }
    const double delta = state[static_cast<std::size_t>(*steering_index_)];
    if (*wheelbase_ <= 0.0) {
        return std::nullopt;
    }
    return speed * std::cos(*beta) * std::tan(delta) / *wheelbase_;
}

std::optional<double> LowSpeedSafety::kinematic_slip(const std::vector<double>& state, double)
    const
{
    (void)state;
    return kinematic_beta(state);
}

std::optional<double> LowSpeedSafety::kinematic_lateral_velocity(const std::vector<double>& state, double speed) const
{
    const auto beta = kinematic_beta(state);
    if (!beta.has_value()) {
        return std::nullopt;
    }
    return speed * std::sin(*beta);
}

std::optional<double> LowSpeedSafety::velocity_slip(const std::vector<double>& state) const
{
    if (!longitudinal_index_.has_value() || !lateral_index_.has_value()) {
        if (slip_index_.has_value() && index_in_bounds(*slip_index_, state)) {
            return state[static_cast<std::size_t>(*slip_index_)];
        }
        return std::nullopt;
    }
    if (!index_in_bounds(*longitudinal_index_, state) || !index_in_bounds(*lateral_index_, state)) {
        return std::nullopt;
    }

    const double longitudinal = state[static_cast<std::size_t>(*longitudinal_index_)];
    const double lateral      = state[static_cast<std::size_t>(*lateral_index_)];
    if (std::abs(longitudinal) <= 1e-9 && std::abs(lateral) <= 1e-9) {
        return 0.0;
    }

    return std::atan2(lateral, longitudinal);
}

double LowSpeedSafety::pre_latch_blend(double speed, const LowSpeedSafetyProfile& profile) const
{
    const double band   = std::max(profile.release_speed - profile.engage_speed, 0.0);
    const double upper  = profile.release_speed + band;
    const double lower  = profile.release_speed;
    if (upper <= lower || speed >= upper) {
        return 0.0;
    }
    if (speed <= lower) {
        return 1.0;
    }
    const double ratio = (upper - speed) / (upper - lower);
    return std::clamp(ratio, 0.0, 1.0);
}

double LowSpeedSafety::scaled_limit(double limit, double speed, const LowSpeedSafetyProfile& profile) const
{
    if (speed >= profile.release_speed || profile.release_speed <= 0.0) {
        return limit;
    }
    const double ratio = std::clamp(speed / std::max(profile.release_speed, 1e-6), 0.0, 1.0);
    const double min_limit = std::max(config_.stop_speed_epsilon, 1e-6);
    return std::clamp(limit * ratio, min_limit, limit);
}

} // namespace velox::simulation
