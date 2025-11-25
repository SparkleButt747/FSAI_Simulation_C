#include "simulation/loss_of_control_detector.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "common/errors.hpp"

namespace velox::simulation {

void MetricThreshold::validate(const char* name) const
{
    if (!(magnitude_threshold > 0.0)) {
        std::ostringstream oss;
        oss << "MetricThreshold." << name << ".magnitude_threshold must be positive";
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
    if (!(rate_threshold > 0.0)) {
        std::ostringstream oss;
        oss << "MetricThreshold." << name << ".rate_threshold must be positive";
        throw ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    }
}

void LossOfControlDetectorConfig::validate() const
{
    yaw_rate.validate("yaw_rate");
    slip_angle.validate("slip_angle");
    lateral_accel.validate("lateral_accel");
    slip_ratio.validate("slip_ratio");
}

LossOfControlDetector::LossOfControlDetector(LossOfControlDetectorConfig config)
    : config_(std::move(config))
{
    config_.validate();
}

void LossOfControlDetector::reset()
{
    yaw_rate_state_      = {};
    slip_angle_state_    = {};
    lateral_accel_state_ = {};
    wheel_states_.clear();
    severity_ = 0.0;
}

double LossOfControlDetector::update(double dt,
                                     double yaw_rate,
                                     double slip_angle,
                                     double lateral_accel,
                                     const std::vector<double>& wheel_slip_ratios)
{
    if (!(dt > 0.0)) {
        throw ::velox::errors::InputError(VELOX_LOC("LossOfControlDetector requires positive dt"));
    }

    if (wheel_states_.size() != wheel_slip_ratios.size()) {
        wheel_states_.assign(wheel_slip_ratios.size(), MetricState{});
    }

    double max_severity = 0.0;
    max_severity        = std::max(max_severity, evaluate_metric(dt, yaw_rate, yaw_rate_state_, config_.yaw_rate));
    max_severity        = std::max(max_severity, evaluate_metric(dt, slip_angle, slip_angle_state_, config_.slip_angle));
    max_severity        =
        std::max(max_severity, evaluate_metric(dt, lateral_accel, lateral_accel_state_, config_.lateral_accel));

    for (std::size_t i = 0; i < wheel_slip_ratios.size(); ++i) {
        max_severity =
            std::max(max_severity, evaluate_metric(dt, wheel_slip_ratios[i], wheel_states_[i], config_.slip_ratio));
    }

    severity_ = max_severity;
    return severity_;
}

double LossOfControlDetector::evaluate_metric(double dt,
                                              double value,
                                              MetricState& state,
                                              const MetricThreshold& limits) const
{
    double severity = 0.0;

    if (state.has_previous) {
        const double delta = std::abs(value - state.value);
        const double rate  = delta / dt;
        const double mag   = std::abs(value);

        if (mag >= limits.magnitude_threshold && rate >= limits.rate_threshold) {
            const double mag_score  = (mag - limits.magnitude_threshold) / limits.magnitude_threshold;
            const double rate_score = (rate - limits.rate_threshold) / limits.rate_threshold;
            severity                = std::max(0.0, 0.5 * mag_score + 0.5 * rate_score);
        }
    }

    state.value        = value;
    state.has_previous = true;
    return severity;
}

} // namespace velox::simulation

