#pragma once

namespace velox::models {
namespace utils {

/**
 * TrailerParameters
 * Class defines all trailer parameters (for on-axle trailer-truck models).
 */
struct TrailerParameters {
    double l{};        // trailer length [m]
    double w{};        // trailer width [m]
    double l_hitch{};  // hitch length [m]
    double l_total{};  // total system length [m]
    double l_wb{};     // trailer wheel base [m]
};

} // namespace utils
} // namespace velox::models
