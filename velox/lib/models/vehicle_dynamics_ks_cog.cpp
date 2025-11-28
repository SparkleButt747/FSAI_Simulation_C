#include "models/vehicle_dynamics_ks_cog.hpp"

#include <cmath>

namespace velox::models {
namespace utils {

std::array<double, 5> vehicle_dynamics_ks_cog(const std::array<double, 5>& x,
                                              const std::array<double, 2>& u_init,
                                              const ::velox::models::VehicleParameters& p)
{
    // wheelbase
    const double l_wb = p.a + p.b;

    // consider steering & acceleration constraints (copy Python semantics)
    std::array<double, 2> u;
    u[0] = steering_constraints(x[2], u_init[0], p.steering);
    u[1] = acceleration_constraints(x[3], u_init[1], p.longitudinal);

    // slip angle (beta) from vehicle kinematics
    const double beta = std::atan(std::tan(x[2]) * p.b / l_wb);

    // system dynamics
    std::array<double, 5> f;
    f[0] = x[3] * std::cos(beta + x[4]);
    f[1] = x[3] * std::sin(beta + x[4]);
    f[2] = u[0];
    f[3] = u[1];
    f[4] = x[3] * std::cos(beta) * std::tan(x[2]) / l_wb;

    return f;
}

} // namespace utils
} // namespace velox::models
