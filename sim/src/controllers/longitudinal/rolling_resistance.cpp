#include "controllers/longitudinal/rolling_resistance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "common/errors.hpp"
namespace velox::controllers::longitudinal {

void RollingResistanceConfig::validate() const
{
    if (!std::isfinite(c_rr)) {
        throw ::velox::errors::ConfigError(VELOX_LOC("rolling_resistance.c_rr must be finite"));
    }
    if (c_rr < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("rolling_resistance.c_rr cannot be negative"));
    }
}

RollingResistance::RollingResistance(RollingResistanceConfig config, double gravity)
    : config_(config)
    , gravity_(gravity)
{
    config_.validate();
    if (!std::isfinite(gravity_) || gravity_ <= 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("gravity must be positive and finite"));
    }
}

double RollingResistance::force(double speed, double normal_force) const noexcept
{
    const double base = config_.c_rr * std::max(0.0, normal_force);
    if (speed == 0.0) {
        return 0.0;
    }
    return -std::copysign(base, speed);
}

} // namespace velox::controllers::longitudinal
