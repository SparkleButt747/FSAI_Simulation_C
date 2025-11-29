#pragma once

#include <cmath>

#include "models/longitudinal_parameters.hpp"

namespace velox::models {
namespace utils {

/**
 * Adjusts the acceleration based on acceleration constraints.
 */
inline double acceleration_constraints(double velocity,
                                       double acceleration,
                                       const LongitudinalParameters& p)
{
    // positive acceleration limit
    double posLimit;
    if (velocity > p.v_switch && p.v_switch > 0.0) {
        posLimit = p.a_max * p.v_switch / velocity;
    } else {
        posLimit = p.a_max;
    }

    // acceleration limit reached?
    if ((velocity <= p.v_min && acceleration <= 0.0) ||
        (velocity >= p.v_max && acceleration >= 0.0))
    {
        acceleration = 0.0;
    } else if (acceleration <= -p.a_max) {
        acceleration = -p.a_max;
    } else if (acceleration >= posLimit) {
        acceleration = posLimit;
    }

    return acceleration;
}

/**
 * Adjusts jerk_dot if jerk limit or input bounds are reached.
 */
inline double jerk_dot_constraints(double jerk_dot,
                                   double jerk,
                                   const LongitudinalParameters& p)
{
    // input constraints for jerk_dot: adjusts jerk_dot if jerk limit or input bounds are reached
    if ((jerk_dot < 0.0 && jerk <= -p.j_max) ||
        (jerk_dot > 0.0 && jerk >=  p.j_max))
    {
        // jerk limit reached
        jerk_dot = 0.0;
    } else if (std::fabs(jerk_dot) >= p.j_dot_max) {
        // input bounds reached
        // NOTE: mirrors Python behaviour: sign is discarded when saturated.
        jerk_dot = p.j_dot_max;
    }
    return jerk_dot;
}

} // namespace utils
} // namespace velox::models
