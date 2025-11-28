#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace velox::simulation {

enum class ModelType {
    MB  = 0,
    ST  = 1,
    STD = 2
};

struct ModelTimingInfo {
    float nominal_dt = 0.0f;
    float max_dt     = 0.0f;
};

inline constexpr float kMinStableDt = 0.001f;

namespace detail {
inline constexpr std::array<ModelTimingInfo, 3> kModelTimings = {
    ModelTimingInfo{0.005f, 0.005f}, // MB
    ModelTimingInfo{0.010f, 0.020f}, // ST
    ModelTimingInfo{0.010f, 0.010f}  // STD
};

inline constexpr std::array<const char*, 3> kModelNames = {
    "Multi-body (vehicle_dynamics_mb)",
    "ST (vehicle_dynamics_st)",
    "STD (vehicle_dynamics_std)"
};
} // namespace detail

inline constexpr const ModelTimingInfo& model_timing(ModelType type)
{
    const std::size_t index = static_cast<std::size_t>(type);
    return detail::kModelTimings[index];
}

inline constexpr const char* model_display_name(ModelType type)
{
    const std::size_t index = static_cast<std::size_t>(type);
    return detail::kModelNames[index];
}

class ModelTiming {
public:
    struct StepSchedule {
        double              requested_dt{0.0};
        double              clamped_dt{0.0};
        std::vector<double> substeps{};
        bool                clamped_to_min{false};
        bool                used_substeps{false};

        double total_duration() const;
    };

    explicit ModelTiming(ModelTimingInfo info = {}, double start_time_s = 0.0);

    void reset(double start_time_s = 0.0);

    [[nodiscard]] const ModelTimingInfo& info() const { return info_; }
    [[nodiscard]] double cumulative_time() const { return cumulative_time_s_; }

    [[nodiscard]] StepSchedule plan_steps(double requested_dt) const;
    void                       record_step(double dt);

private:
    ModelTimingInfo info_{};
    double          cumulative_time_s_{0.0};
};

} // namespace velox::simulation
