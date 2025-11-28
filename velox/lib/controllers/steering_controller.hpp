#pragma once

#include <string>

#include "models/steering_parameters.hpp"

namespace velox::controllers {

using velox::models::utils::SteeringParameters;

struct SteeringConfig {
    struct Wheel {
        double max_angle{0.0};
        double max_rate{0.0};
        double nudge_angle{0.0};
        double centering_stiffness{0.0};
        double centering_deadband{0.0};

        void validate() const;
    };

    struct Final {
        double min_angle{0.0};
        double max_angle{0.0};
        double max_rate{0.0};
        double actuator_time_constant{0.0};
        double smoothing_time_constant{0.0};

        void validate() const;
    };

    Wheel wheel{};
    Final final{};

    void validate() const;

};

class SteeringWheel {
public:
    SteeringWheel() = default;
    SteeringWheel(const SteeringConfig::Wheel& cfg,
                  const SteeringParameters& limits);

    struct Output {
        double target_angle{0.0};
        double angle{0.0};
        double rate{0.0};
    };

    Output update(double nudge, double dt);
    void   reset(double angle = 0.0);

    const Output& last_output() const { return last_output_; }
    const SteeringConfig::Wheel& config() const noexcept { return cfg_; }

private:
    SteeringConfig::Wheel cfg_{};
    SteeringParameters    limits_{};
    double                angle_{0.0};
    Output                last_output_{};

    [[nodiscard]] double max_left() const;
    [[nodiscard]] double max_right() const;
};

class FinalSteerController {
public:
    FinalSteerController() = default;
    FinalSteerController(const SteeringConfig::Final& cfg,
                         const SteeringParameters& limits);

    struct Output {
        double filtered_target{0.0};
        double angle{0.0};
        double rate{0.0};
    };

    Output update(double desired_angle, double current_angle, double dt);
    Output update_absolute(double desired_angle, double current_angle, double dt);
    void   reset(double current_angle = 0.0);

    const Output& last_output() const { return last_output_; }
    const SteeringConfig::Final& config() const noexcept { return cfg_; }
    [[nodiscard]] double min_angle() const;
    [[nodiscard]] double max_angle() const;

private:
    SteeringConfig::Final cfg_{};
    SteeringParameters       limits_{};
    double                   filtered_target_{0.0};
    Output                   last_output_{};
};

} // namespace velox::controllers
