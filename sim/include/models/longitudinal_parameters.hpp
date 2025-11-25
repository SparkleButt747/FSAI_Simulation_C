#pragma once

namespace velox::models {
namespace utils {

/**
 * LongitudinalParameters
 * Class defines all parameters related to the longitudinal dynamics.
 */
struct LongitudinalParameters {
    // constraints regarding longitudinal dynamics
    double v_min{};      // minimum velocity [m/s]
    double v_max{};      // maximum velocity [m/s]
    double v_switch{};   // switching velocity [m/s]
    double a_max{};      // maximum absolute acceleration [m/s^2]
    double j_max{};      // maximum longitudinal jerk [m/s^3]
    double j_dot_max{};  // maximum change of longitudinal jerk [m/s^4]
};

} // namespace utils
} // namespace velox::models
