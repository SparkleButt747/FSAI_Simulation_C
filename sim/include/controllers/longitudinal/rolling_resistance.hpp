#pragma once

namespace velox::controllers::longitudinal {

struct RollingResistanceConfig {
    double c_rr = 0.0;

    void validate() const;
};

class RollingResistance {
public:
    RollingResistance(RollingResistanceConfig config, double gravity = 9.81);

    [[nodiscard]] double force(double speed, double normal_force) const noexcept;

    [[nodiscard]] const RollingResistanceConfig& config() const noexcept { return config_; }

private:
    RollingResistanceConfig config_;
    double gravity_;
};

} // namespace velox::controllers::longitudinal
