#pragma once

namespace velox::controllers::longitudinal {

struct BrakeConfig {
    double max_force        = 0.0;
    double max_regen_force  = 0.0;
    double min_regen_speed  = 0.0;

    void validate() const;
};

struct BrakeBlendOutput {
    double regen_force     = 0.0;
    double hydraulic_force = 0.0;
    double total_force     = 0.0;
};

class BrakeController {
public:
    explicit BrakeController(BrakeConfig config);

    [[nodiscard]] BrakeBlendOutput blend(double brake_pedal,
                                         double speed,
                                         double available_regen_force) const noexcept;

    [[nodiscard]] const BrakeConfig& config() const noexcept { return config_; }

private:
    BrakeConfig config_;
};

} // namespace velox::controllers::longitudinal
