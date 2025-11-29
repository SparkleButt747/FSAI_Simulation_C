#include "controllers/longitudinal/brake.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "common/errors.hpp"
namespace velox::controllers::longitudinal {

void BrakeConfig::validate() const
{
    if (!std::isfinite(max_force) || max_force < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("brake.max_force must be non-negative and finite"));
    }
    if (!std::isfinite(max_regen_force) || max_regen_force < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("brake.max_regen_force must be non-negative and finite"));
    }
    if (!std::isfinite(min_regen_speed) || min_regen_speed < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("brake.min_regen_speed must be non-negative and finite"));
    }
}

BrakeController::BrakeController(BrakeConfig config)
    : config_(config)
{
    config_.validate();
}

BrakeBlendOutput BrakeController::blend(double brake_pedal,
                                        double speed,
                                        double available_regen_force) const noexcept
{
    brake_pedal            = std::clamp(brake_pedal, 0.0, 1.0);
    available_regen_force  = std::max(0.0, available_regen_force);

    const double total_force = std::min(config_.max_force, brake_pedal * config_.max_force);

    double regen_capacity = std::min({total_force,
                                      brake_pedal * config_.max_regen_force,
                                      available_regen_force});
    regen_capacity        = std::max(0.0, regen_capacity);

    const double min_regen_speed = std::max(0.0, config_.min_regen_speed);
    double weight                = 1.0;
    if (min_regen_speed > 0.0) {
        const double speed_ratio = std::abs(speed) / min_regen_speed;
        weight                   = std::clamp(speed_ratio, 0.0, 1.0);
    }

    const double regen_force     = regen_capacity * weight;
    const double hydraulic_force = std::max(0.0, total_force - regen_force);

    return BrakeBlendOutput{regen_force, hydraulic_force, total_force};
}

} // namespace velox::controllers::longitudinal
