#pragma once

#include <vector>

namespace velox::simulation {

struct MetricThreshold {
    double magnitude_threshold  = 0.0;
    double rate_threshold       = 0.0;

    void validate(const char* name) const;
};

struct LossOfControlDetectorConfig {
    MetricThreshold yaw_rate{};
    MetricThreshold slip_angle{};
    MetricThreshold lateral_accel{};
    MetricThreshold slip_ratio{};

    void validate() const;
};

class LossOfControlDetector {
public:
    explicit LossOfControlDetector(LossOfControlDetectorConfig config);

    void   reset();
    double severity() const { return severity_; }

    double update(double dt,
                  double yaw_rate,
                  double slip_angle,
                  double lateral_accel,
                  const std::vector<double>& wheel_slip_ratios);

private:
    struct MetricState {
        double value        = 0.0;
        bool   has_previous = false;
    };

    LossOfControlDetectorConfig config_{};
    MetricState                 yaw_rate_state_{};
    MetricState                 slip_angle_state_{};
    MetricState                 lateral_accel_state_{};
    std::vector<MetricState>    wheel_states_{};
    double                      severity_ = 0.0;

    double evaluate_metric(double dt, double value, MetricState& state, const MetricThreshold& limits) const;
};

} // namespace velox::simulation

