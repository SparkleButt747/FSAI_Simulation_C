#include "models/vehiclemodels/vehicle_dynamics_st.hpp"

#include <cmath>
#include <vector>

#include "common/errors.hpp"

#include "models/acceleration_constraints.hpp"
#include "models/steering_constraints.hpp"
#include "models/vehicle_dynamics_ks_cog.hpp"
#include "models/tire_model.hpp"

namespace velox::models {

std::vector<double> vehicle_dynamics_st(const std::vector<double>& x,
                                        const std::vector<double>& uInit,
                                        const VehicleParameters& p)
{
    if (x.size() != 7 || uInit.size() != 2) {
        throw ::velox::errors::SimulationError(
            VELOX_LOC("vehicle_dynamics_st: expected x.size()==7 and uInit.size()==2"));
    }

    // set gravity constant
    const double g = 9.81; // [m/s^2]

    // create equivalent bicycle parameters
    const double mu   = p.tire.p_dy1;
    const double C_Sf = -p.tire.p_ky1 / p.tire.p_dy1;
    const double C_Sr = -p.tire.p_ky1 / p.tire.p_dy1;
    const double lf   = p.a;
    const double lr   = p.b;
    const double h    = p.h_s;
    const double m    = p.m;
    const double I    = p.I_z;

    // inputs:
    // u1 = steering angle velocity of front wheels
    // u2 = longitudinal acceleration

    // steering & acceleration constraints
    std::vector<double> u(2);
    u[0] = utils::steering_constraints(x[2], uInit[0], p.steering);
    u[1] = utils::acceleration_constraints(x[3], uInit[1], p.longitudinal);

    std::vector<double> f;

    // switch to kinematic model for small velocities
    if (std::fabs(x[3]) < 0.1) {
        // Use kinematic model with reference point at center of mass
        const double lwb = p.a + p.b;

        // system dynamics
        std::array<double, 5> x_ks{
            x[0], x[1], x[2], x[3], x[4]
        };
        std::array<double, 2> u_arr{u[0], u[1]};
        auto f_ks = utils::vehicle_dynamics_ks_cog(x_ks, u_arr, p);

        f.reserve(7);
        f.push_back(f_ks[0]);
        f.push_back(f_ks[1]);
        f.push_back(f_ks[2]);
        f.push_back(f_ks[3]);
        f.push_back(f_ks[4]);

        // derivative of slip angle and yaw rate
        const double tan_delta = std::tan(x[2]);
        const double cos_delta = std::cos(x[2]);
        const double cos_delta_sq = cos_delta * cos_delta;
        const double term = tan_delta * tan_delta * p.b / lwb;

        const double d_beta = (p.b * u[0]) /
                              (lwb * cos_delta_sq * (1.0 + term * term));
        const double dd_psi = 1.0 / lwb *
                              (u[1] * std::cos(x[6]) * tan_delta -
                               x[3] * std::sin(x[6]) * d_beta * tan_delta +
                               x[3] * std::cos(x[6]) * u[0] / cos_delta_sq);

        f.push_back(dd_psi);
        f.push_back(d_beta);
    } else {
        // dynamic single-track model

        f.reserve(7);
        // position
        f.push_back(x[3] * std::cos(x[6] + x[4]));
        f.push_back(x[3] * std::sin(x[6] + x[4]));
        // steering and longitudinal acceleration
        f.push_back(u[0]);
        f.push_back(u[1]);

        // yaw rate derivative (x6_dot)
        const double term1 = -mu * m / (x[3] * I * (lr + lf)) *
            (lf * lf * C_Sf * (g * lr - u[1] * h) +
             lr * lr * C_Sr * (g * lf + u[1] * h)) * x[5];
        const double term2 = mu * m / (I * (lr + lf)) *
            (lr * C_Sr * (g * lf + u[1] * h) -
             lf * C_Sf * (g * lr - u[1] * h)) * x[6];
        const double term3 = mu * m / (I * (lr + lf)) *
            lf * C_Sf * (g * lr - u[1] * h) * x[2];

        f.push_back(x[5]);       // yaw angle derivative: psi_dot
        f.push_back(term1 + term2 + term3);

        // slip angle derivative (x7_dot)
        const double coeff = (mu / (x[3] * x[3] * (lr + lf)) *
                              (C_Sr * (g * lf + u[1] * h) * lr -
                               C_Sf * (g * lr - u[1] * h) * lf) - 1.0);
        const double c2 = -mu / (x[3] * (lr + lf)) *
                          (C_Sr * (g * lf + u[1] * h) +
                           C_Sf * (g * lr - u[1] * h));
        const double c3 = mu / (x[3] * (lr + lf)) *
                          (C_Sf * (g * lr - u[1] * h));

        f.push_back(coeff * x[5] + c2 * x[6] + c3 * x[2]);
    }

    return f;
}

} // namespace velox::models
