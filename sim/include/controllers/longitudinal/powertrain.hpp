#pragma once

namespace velox::controllers::longitudinal {

struct PowertrainConfig {
    double max_drive_torque      = 0.0;
    double max_regen_torque      = 0.0;
    double max_power             = 0.0;
    double drive_efficiency      = 1.0;
    double regen_efficiency      = 1.0;
    double min_soc               = 0.0;
    double max_soc               = 1.0;
    double initial_soc           = 0.0;
    double battery_capacity_kwh  = 0.0;

    void validate() const;
};

struct PowertrainOutput {
    double total_torque = 0.0;
    double drive_torque = 0.0;
    double regen_torque = 0.0;
    double mechanical_power = 0.0; // +: to wheels, -: regen
    double battery_power    = 0.0; // +: discharge, -: charge
};

class Powertrain {
public:
    Powertrain(PowertrainConfig config, double wheel_radius);

    [[nodiscard]] double available_drive_torque(double speed) const noexcept;

    [[nodiscard]] double available_regen_torque(double speed) const noexcept;

    [[nodiscard]] PowertrainOutput step(double throttle,
                                        double regen_torque_request,
                                        double speed,
                                        double dt);

    void reset() noexcept;

    [[nodiscard]] double soc() const noexcept { return soc_; }
    [[nodiscard]] const PowertrainConfig& config() const noexcept { return config_; }

private:
    [[nodiscard]] double torque_power_limited(double speed) const noexcept;

    PowertrainConfig config_;
    double wheel_radius_;
    double capacity_joules_;
    double soc_;
};

} // namespace velox::controllers::longitudinal
