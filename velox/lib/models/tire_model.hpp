#pragma once

#include <utility>
#include "models/tireParameters.hpp"

namespace velox::models {
namespace utils {

/**
 * sign function matching Python behaviour.
 */
double sign(double x);

// longitudinal tire forces
double formula_longitudinal(double kappa,
                            double gamma,
                            double F_z,
                            const TireParameters& p);

// lateral tire forces: returns {F_y, mu_y}
std::pair<double, double> formula_lateral(double alpha,
                                          double gamma,
                                          double F_z,
                                          const TireParameters& p);

// longitudinal tire forces for combined slip
double formula_longitudinal_comb(double kappa,
                                 double alpha,
                                 double F0_x,
                                 const TireParameters& p);

// lateral tire forces for combined slip
double formula_lateral_comb(double kappa,
                            double alpha,
                            double gamma,
                            double mu_y,
                            double F_z,
                            double F0_y,
                            const TireParameters& p);

} // namespace utils
} // namespace velox::models
