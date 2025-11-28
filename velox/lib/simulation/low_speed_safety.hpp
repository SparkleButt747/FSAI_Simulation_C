#pragma once

#include <optional>
#include <vector>

namespace velox::simulation {

struct LowSpeedSafetyProfile {
    double engage_speed     = 0.0;
    double release_speed    = 0.0;
    double yaw_rate_limit   = 0.0;
    double slip_angle_limit = 0.0;

    void validate(const char* name) const;
};

struct LowSpeedSafetyConfig {
    LowSpeedSafetyProfile normal{};
    LowSpeedSafetyProfile drift{};
    double                stop_speed_epsilon = 0.0;
    bool                  drift_enabled      = false;

    void validate() const;
    [[nodiscard]] const LowSpeedSafetyProfile& active_profile(bool drift_mode) const
    {
        return drift_mode ? drift : normal;
    }
};

enum class SafetyStage
{
    Normal,
    Transition,
    Emergency
};

struct LowSpeedSafetyStatus
{
    double      severity         = 0.0;
    double      transition_blend = 0.0;
    bool        drift_mode       = false;
    bool        detector_forced  = false;
    bool        latch_active     = false;
    SafetyStage stage            = SafetyStage::Normal;
};

class LowSpeedSafety {
public:
    LowSpeedSafety(const LowSpeedSafetyConfig& config,
                   std::optional<int> longitudinal_index,
                   std::optional<int> lateral_index,
                   std::optional<int> yaw_rate_index,
                   std::optional<int> slip_index,
                   std::vector<int> wheel_speed_indices = {},
                   std::optional<int> steering_index = std::nullopt,
                   std::optional<double> wheelbase = std::nullopt,
                   std::optional<double> rear_length = std::nullopt);

    void reset();
    bool engaged() const { return engaged_; }
    bool drift_enabled() const { return drift_enabled_; }
    void set_drift_enabled(bool enabled) { drift_enabled_ = enabled; }

    [[nodiscard]] LowSpeedSafetyStatus status(const std::vector<double>& state, double speed) const;

    void apply(std::vector<double>& state, double speed, bool update_latch = true);
    [[nodiscard]] std::optional<int> longitudinal_index() const { return longitudinal_index_; }

private:
    struct MonitoredMetrics;
    struct SafetyDecision;

    LowSpeedSafetyConfig config_{};
    bool                 drift_enabled_ = false;
    std::optional<int> longitudinal_index_;
    std::optional<int> lateral_index_;
    std::optional<int> yaw_rate_index_;
    std::optional<int> slip_index_;
    std::vector<int>   wheel_speed_indices_;
    std::optional<int> steering_index_;
    std::optional<double> wheelbase_;
    std::optional<double> rear_length_;
    bool engaged_ = false;

    static bool index_in_bounds(int index, const std::vector<double>& state);
    static double clamp(double value, double min_value, double max_value);

    std::optional<double> kinematic_beta(const std::vector<double>& state) const;
    std::optional<double> kinematic_yaw_rate(const std::vector<double>& state, double speed) const;
    std::optional<double> kinematic_slip(const std::vector<double>& state, double speed) const;
    std::optional<double> kinematic_lateral_velocity(const std::vector<double>& state, double speed) const;
    std::optional<double> velocity_slip(const std::vector<double>& state) const;
    double                pre_latch_blend(double speed, const LowSpeedSafetyProfile& profile) const;
    double                scaled_limit(double limit, double speed, const LowSpeedSafetyProfile& profile) const;
    MonitoredMetrics      monitor(const std::vector<double>& state, double speed, const LowSpeedSafetyProfile& profile) const;
    SafetyDecision        decide(const MonitoredMetrics& metrics, const LowSpeedSafetyProfile& profile, bool update_latch);
    void                  clamp_state(std::vector<double>&        state,
                                      const MonitoredMetrics&     metrics,
                                      const SafetyDecision&       decision,
                                      const LowSpeedSafetyProfile& profile);
};

} // namespace velox::simulation
