#pragma once

namespace velox::models {
namespace utils {

/**
 * SteeringParameters
 * Class defines all steering related parameters.
 */
struct SteeringParameters {
    // constraints regarding steering
    double min{};              // minimum steering angle [rad]
    double max{};              // maximum steering angle [rad]
    double v_min{};            // minimum steering velocity [rad/s]
    double v_max{};            // maximum steering velocity [rad/s]
    double kappa_dot_max{};    // maximum curvature rate
    double kappa_dot_dot_max{};// maximum curvature rate rate
};

} // namespace utils
} // namespace velox::models
