#include "models/vehiclemodels/vehicle_dynamics_std.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "common/errors.hpp"
#include "models/steering_constraints.hpp"
#include "models/acceleration_constraints.hpp"
#include "models/vehicle_dynamics_ks_cog.hpp"
#include "models/tire_model.hpp"

namespace velox::models {

namespace {
constexpr double kWheelRelaxationTime = 0.02;
constexpr double kDefaultStdStep = 0.01;
} // namespace

using utils::steering_constraints;
using utils::acceleration_constraints;
using utils::vehicle_dynamics_ks_cog;
using utils::formula_longitudinal;
using utils::formula_lateral;
using utils::formula_longitudinal_comb;
using utils::formula_lateral_comb;

std::vector<double> vehicle_dynamics_std(const std::vector<double>& x,
                                         const std::vector<double>& u_init,
                                         const VehicleParameters& p,
                                         double dt)
{
    // Sanity: expect 9 states and 2 inputs
    if (x.size() != 9 || u_init.size() != 2) {
        throw ::velox::errors::SimulationError(
            VELOX_LOC("vehicle_dynamics_std: expected x.size()==9 and u_init.size()==2"));
    }

    // ---------------------------------------------------------------------
    // Constants and basic parameters
    // ---------------------------------------------------------------------
    constexpr double g = 9.81;  // [m/s^2]

    const double lf  = p.a;
    const double lr  = p.b;
    const double lwb = p.a + p.b;
    const double m   = p.m;
    const double I   = p.I_z;

    // mix models parameters
    const double v_s   = 0.2;
    const double v_b   = 0.05;
    const double v_min = v_s / 2.0;

    // ---------------------------------------------------------------------
    // Inputs (steering + longitudinal accel with constraints)
    // ---------------------------------------------------------------------
    std::vector<double> u(2);
    // steering and acceleration constraints (copy Python semantics)
    u[0] = steering_constraints(x[2], u_init[0], p.steering);
    u[1] = acceleration_constraints(x[3], u_init[1], p.longitudinal);

    // ---------------------------------------------------------------------
    // Lateral tire slip angles
    // Python:
    // alpha_f = atan((x[3] * sin(x[6]) + x[5] * lf) / (x[3] * cos(x[6]))) - x[2] if x[3] > v_min else 0
    // alpha_r = atan((x[3] * sin(x[6]) - x[5] * lr) / (x[3] * cos(x[6]))) if x[3] > v_min else 0
    // ---------------------------------------------------------------------
    double alpha_f = 0.0;
    double alpha_r = 0.0;
    if (x[3] > v_min) {
        alpha_f = std::atan((x[3] * std::sin(x[6]) + x[5] * lf) /
                            (x[3] * std::cos(x[6]))) - x[2];
        alpha_r = std::atan((x[3] * std::sin(x[6]) - x[5] * lr) /
                            (x[3] * std::cos(x[6])));
    }

    // ---------------------------------------------------------------------
    // Vertical tire forces
    // Python:
    // F_zf = m * (-u[1] * p.h_s + g * lr) / (lr + lf)
    // F_zr = m * ( u[1] * p.h_s + g * lf) / (lr + lf)
    // ---------------------------------------------------------------------
    const double F_zf = m * (-u[1] * p.h_s + g * lr) / (lr + lf);
    const double F_zr = m * ( u[1] * p.h_s + g * lf) / (lr + lf);

    // ---------------------------------------------------------------------
    // Front and rear tire speeds
    // Python:
    // u_wf = max(0, x[3]*cos(x[6])*cos(x[2]) + (x[3]*sin(x[6])+p.a*x[5])*sin(x[2]))
    // u_wr = max(0, x[3]*cos(x[6]))
    // ---------------------------------------------------------------------
    const double u_wf = std::max(
        0.0,
        x[3] * std::cos(x[6]) * std::cos(x[2]) +
        (x[3] * std::sin(x[6]) + p.a * x[5]) * std::sin(x[2])
    );
    const double u_wr = std::max(0.0, x[3] * std::cos(x[6]));

    const double omega_f_state = std::max(0.0, x[7]);
    const double omega_r_state = std::max(0.0, x[8]);

    // ---------------------------------------------------------------------
    // Longitudinal tire slip
    // ---------------------------------------------------------------------
    const double s_f = 1.0 - p.R_w * omega_f_state / std::max(u_wf, v_min);
    const double s_r = 1.0 - p.R_w * omega_r_state / std::max(u_wr, v_min);

    // ---------------------------------------------------------------------
    // Tire forces (Pacejka) — pure slip
    // ---------------------------------------------------------------------
    const double F0_xf = formula_longitudinal(s_f, 0.0, F_zf, p.tire);
    const double F0_xr = formula_longitudinal(s_r, 0.0, F_zr, p.tire);

    auto lat_f = formula_lateral(alpha_f, 0.0, F_zf, p.tire);
    const double F0_yf = lat_f.first;
    const double mu_yf = lat_f.second;

    auto lat_r = formula_lateral(alpha_r, 0.0, F_zr, p.tire);
    const double F0_yr = lat_r.first;
    const double mu_yr = lat_r.second;

    // ---------------------------------------------------------------------
    // Tire forces (Pacejka) — combined slip
    // ---------------------------------------------------------------------
    const double F_xf = formula_longitudinal_comb(s_f, alpha_f, F0_xf, p.tire);
    const double F_xr = formula_longitudinal_comb(s_r, alpha_r, F0_xr, p.tire);

    const double F_yf = formula_lateral_comb(s_f, alpha_f, 0.0, mu_yf, F_zf, F0_yf, p.tire);
    const double F_yr = formula_lateral_comb(s_r, alpha_r, 0.0, mu_yr, F_zr, F0_yr, p.tire);

    // ---------------------------------------------------------------------
    // Convert acceleration input to brake and engine torque
    // Python:
    // if u[1] > 0:
    //     T_B = 0.0
    //     T_E = m * p.R_w * u[1]
    // else:
    //     T_B = m * p.R_w * u[1]
    //     T_E = 0.
    // ---------------------------------------------------------------------
    double T_B = 0.0;
    double T_E = 0.0;
    if (u[1] > 0.0) {
        T_B = 0.0;
        T_E = m * p.R_w * u[1];
    } else {
        T_B = m * p.R_w * u[1];
        T_E = 0.0;
    }

    // ---------------------------------------------------------------------
    // System dynamics (dynamic model)
    // Python:
    // d_v = 1/m * (-F_yf*sin(x[2]-x[6]) + F_yr*sin(x[6]) + F_xr*cos(x[6]) + F_xf*cos(x[2]-x[6]))
    // dd_psi = 1/I * (F_yf*cos(x[2])*lf - F_yr*lr + F_xf*sin(x[2])*lf)
    // d_beta = -x[5] + 1/(m*x[3])*(F_yf*cos(x[2]-x[6]) + F_yr*cos(x[6]) - F_xr*sin(x[6]) + F_xf*sin(x[2]-x[6]))
    // ---------------------------------------------------------------------
    const double d_v = (1.0 / m) *
        (-F_yf * std::sin(x[2] - x[6]) +
          F_yr * std::sin(x[6]) +
          F_xr * std::cos(x[6]) +
          F_xf * std::cos(x[2] - x[6]));

    const double dd_psi = (1.0 / I) *
        (F_yf * std::cos(x[2]) * lf -
         F_yr * lr +
         F_xf * std::sin(x[2]) * lf);

    double d_beta = 0.0;
    if (x[3] > v_min) {
        d_beta = -x[5] +
            (1.0 / (m * x[3])) *
            (F_yf * std::cos(x[2] - x[6]) +
             F_yr * std::cos(x[6]) -
             F_xr * std::sin(x[6]) +
             F_xf * std::sin(x[2] - x[6]));
    }

    // ---------------------------------------------------------------------
    // Wheel dynamics (negative wheel spin forbidden)
    // Python:
    // d_omega_f = ... if x[7] >= 0 else 0; x[7] = max(0, x[7])
    // d_omega_r = ... if x[8] >= 0 else 0; x[8] = max(0, x[8])
    // ---------------------------------------------------------------------
    const bool front_negative = x[7] < 0.0;
    const bool rear_negative  = x[8] < 0.0;

    double d_omega_f = 0.0;
    double d_omega_r = 0.0;

    if (!front_negative) {
        d_omega_f = (1.0 / p.I_y_w) *
            (-p.R_w * F_xf + p.T_sb * T_B + p.T_se * T_E);
        if (omega_f_state <= 0.0 && d_omega_f < 0.0) {
            d_omega_f = 0.0;
        }
    }

    if (!rear_negative) {
        d_omega_r = (1.0 / p.I_y_w) *
            (-p.R_w * F_xr + (1.0 - p.T_sb) * T_B + (1.0 - p.T_se) * T_E);
        if (omega_r_state <= 0.0 && d_omega_r < 0.0) {
            d_omega_r = 0.0;
        }
    }

    // ---------------------------------------------------------------------
    // Kinematic model (for blending at low speeds)
    // Python:
    // x_ks = [x[0], x[1], x[2], x[3], x[4]]
    // f_ks = vehicle_dynamics_ks_cog(x_ks, u, p)
    // ...
    // d_beta_ks, dd_psi_ks, d_omega_f_ks, d_omega_r_ks
    // ---------------------------------------------------------------------
    std::array<double, 5> x_ks{
        x[0], x[1], x[2], x[3], x[4]
    };
    std::array<double, 2> u_arr{
        u[0], u[1]
    };

    const auto f_ks = vehicle_dynamics_ks_cog(x_ks, u_arr, p);

    const double tan_delta = std::tan(x[2]);
    const double cos_delta = std::cos(x[2]);
    const double cos_delta_sq = cos_delta * cos_delta;
    const double term = tan_delta * tan_delta * p.b / lwb;

    const double d_beta_ks = (p.b * u[0]) /
        (lwb * cos_delta_sq * (1.0 + term * term));

    const double dd_psi_ks = (1.0 / lwb) *
        (u[1] * std::cos(x[6]) * std::tan(x[2]) -
         x[3] * std::sin(x[6]) * d_beta_ks * std::tan(x[2]) +
         x[3] * std::cos(x[6]) * u[0] / cos_delta_sq);

    const double omega_f_clamped = front_negative ? 0.0 : omega_f_state;
    const double omega_r_clamped = rear_negative  ? 0.0 : omega_r_state;

    static_cast<void>(dt);  // Python gain is independent of the integration step
    const double inv_tau = 1.0 / kWheelRelaxationTime;
    const double d_omega_f_ks = inv_tau * (u_wf / p.R_w - omega_f_clamped);
    const double d_omega_r_ks = inv_tau * (u_wr / p.R_w - omega_r_clamped);

    // ---------------------------------------------------------------------
    // Weights for mixing both models
    // Python:
    // w_std = 0.5 * (tanh((x[3] - v_s)/v_b) + 1)
    // w_ks  = 1 - w_std
    // ---------------------------------------------------------------------
    const double w_std = 0.5 * (std::tanh((x[3] - v_s) / v_b) + 1.0);
    const double w_ks  = 1.0 - w_std;

    // ---------------------------------------------------------------------
    // Output vector f (mixed dynamic + kinematic)
    // Python:
    // f = [ x[3]*cos(x[6] + x[4]),
    //       x[3]*sin(x[6] + x[4]),
    //       u[0],
    //       w_std*d_v + w_ks*f_ks[3],
    //       w_std*x[5] + w_ks*f_ks[4],
    //       w_std*dd_psi + w_ks*dd_psi_ks,
    //       w_std*d_beta + w_ks*d_beta_ks,
    //       w_std*d_omega_f + w_ks*d_omega_f_ks,
    //       w_std*d_omega_r + w_ks*d_omega_r_ks ]
    // ---------------------------------------------------------------------
    std::vector<double> f(9);
    f[0] = x[3] * std::cos(x[6] + x[4]);
    f[1] = x[3] * std::sin(x[6] + x[4]);
    f[2] = u[0];
    f[3] = w_std * d_v        + w_ks * f_ks[3];
    f[4] = w_std * x[5]       + w_ks * f_ks[4];
    f[5] = w_std * dd_psi     + w_ks * dd_psi_ks;
    f[6] = w_std * d_beta     + w_ks * d_beta_ks;
    f[7] = w_std * d_omega_f  + w_ks * d_omega_f_ks;
    f[8] = w_std * d_omega_r  + w_ks * d_omega_r_ks;

    return f;
}

std::vector<double> vehicle_dynamics_std(const std::vector<double>& x,
                                         const std::vector<double>& u_init,
                                         const VehicleParameters& p)
{
    return vehicle_dynamics_std(x, u_init, p, kDefaultStdStep);
}

} // namespace velox::models
