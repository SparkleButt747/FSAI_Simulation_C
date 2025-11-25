#pragma once

#include <optional>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"

namespace velox {
namespace controllers {
class SteeringWheel;
class FinalSteerController;

namespace longitudinal {
struct DriverIntent;
struct PowertrainConfig;
} // namespace longitudinal

} // namespace controllers

namespace models {
struct VehicleParameters;
} // namespace models

namespace simulation {

enum class ControlMode {
    Keyboard,
    Direct,
};

struct UserInputLimits {
    double min_throttle{0.0};
    double max_throttle{1.0};
    double min_brake{0.0};
    double max_brake{1.0};
    double min_steering_nudge{-1.0};
    double max_steering_nudge{1.0};
    double min_drift_toggle{0.0};
    double max_drift_toggle{1.0};

    double min_steering_angle{0.0};
    double max_steering_angle{0.0};

    std::vector<double> min_axle_torque{};
    std::vector<double> max_axle_torque{};

    static UserInputLimits from_vehicle(const controllers::SteeringWheel* steering_wheel,
                                        const controllers::FinalSteerController* final_steer,
                                        const models::VehicleParameters& params,
                                        const controllers::longitudinal::PowertrainConfig* powertrain_cfg);
};

struct UserInput {
    ControlMode                             control_mode{ControlMode::Keyboard};
    controllers::longitudinal::DriverIntent longitudinal{};
    double                                  steering_nudge = 0.0;
    double                                  steering_angle{0.0};
    std::vector<double>                     axle_torques{}; // Front axle precedes rear for AWD; single entry for FWD/RWD.
    std::optional<double>                   drift_toggle{};
    double                                  timestamp{0.0};
    double                                  dt{0.0};

    [[nodiscard]] UserInput clamped(const UserInputLimits& limits = {}) const;
    void validate(const UserInputLimits& limits = {}) const;
};

// Transmission/gear inputs are intentionally omitted to keep the interface EV-first.
// If an ICE powertrain is added in the future, prefer introducing a separate
// powertrain control block rather than surfacing raw gearbox state here.

inline constexpr UserInputLimits kDefaultUserInputLimits{};

} // namespace simulation
} // namespace velox

