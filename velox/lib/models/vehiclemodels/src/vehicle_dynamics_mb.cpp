#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "models/acceleration_constraints.hpp"
#include "models/steering_constraints.hpp"
#include "models/tire_model.hpp"
#include "models/vehicle_dynamics_ks_cog.hpp"

#include "common/errors.hpp"

namespace velox::models {

std::vector<double> vehicle_dynamics_mb(const std::vector<double>& x,
                                        const std::vector<double>& uInit,
                                        const VehicleParameters& p)
{
    if (x.size() != 29 || uInit.size() != 2) {
        throw ::velox::errors::SimulationError(
            VELOX_LOC("vehicle_dynamics_mb: expected x.size()==29 and uInit.size()==2"));
    }

    // set gravity constant
    const double g = 9.81;  // [m/s^2]

    // inputs:
    // u1 = steering angle velocity of front wheels
    // u2 = acceleration

    // consider steering constraints
    std::vector<double> u(2);
    u[0] = utils::steering_constraints(x[2], uInit[0], p.steering);
    // consider acceleration constraints
    u[1] = utils::acceleration_constraints(x[3], uInit[1], p.longitudinal);

    // compute slip angle at cg
    // switch to kinematic model for small velocities
    double beta;
    if (std::fabs(x[3]) < 0.1) {
        beta = 0.0;
    } else {
        beta = std::atan(x[10] / x[3]);
    }
    const double vel = std::sqrt(x[3] * x[3] + x[10] * x[10]);

    // vertical tire forces
    const double F_z_LF = (x[16] + p.R_w * (std::cos(x[13]) - 1.0) - 0.5 * p.T_f * std::sin(x[13])) * p.K_zt;
    const double F_z_RF = (x[16] + p.R_w * (std::cos(x[13]) - 1.0) + 0.5 * p.T_f * std::sin(x[13])) * p.K_zt;
    const double F_z_LR = (x[21] + p.R_w * (std::cos(x[18]) - 1.0) - 0.5 * p.T_r * std::sin(x[18])) * p.K_zt;
    const double F_z_RR = (x[21] + p.R_w * (std::cos(x[18]) - 1.0) + 0.5 * p.T_r * std::sin(x[18])) * p.K_zt;

    // obtain individual tire speeds
    double u_w_lf = (x[3] + 0.5 * p.T_f * x[5]) * std::cos(x[2]) +
                    (x[10] + p.a * x[5]) * std::sin(x[2]);
    double u_w_rf = (x[3] - 0.5 * p.T_f * x[5]) * std::cos(x[2]) +
                    (x[10] + p.a * x[5]) * std::sin(x[2]);
    double u_w_lr = x[3] + 0.5 * p.T_r * x[5];
    double u_w_rr = x[3] - 0.5 * p.T_r * x[5];

    // negative wheel spin forbidden (wheel longitudinal speed)
    if (u_w_lf < 0.0) u_w_lf = 0.0;
    if (u_w_rf < 0.0) u_w_rf = 0.0;
    if (u_w_lr < 0.0) u_w_lr = 0.0;
    if (u_w_rr < 0.0) u_w_rr = 0.0;

    // compute longitudinal slip
    // switch to kinematic model for small velocities
    double s_lf, s_rf, s_lr, s_rr;
    if (std::fabs(x[3]) < 0.1) {
        s_lf = 0.0;
        s_rf = 0.0;
        s_lr = 0.0;
        s_rr = 0.0;
    } else {
        const double omega_lf = std::max(0.0, x[23]);
        const double omega_rf = std::max(0.0, x[24]);
        const double omega_lr = std::max(0.0, x[25]);
        const double omega_rr = std::max(0.0, x[26]);

        const double denom_lf = std::max(u_w_lf, 1e-6);
        const double denom_rf = std::max(u_w_rf, 1e-6);
        const double denom_lr = std::max(u_w_lr, 1e-6);
        const double denom_rr = std::max(u_w_rr, 1e-6);

        s_lf = 1.0 - p.R_w * omega_lf / denom_lf;
        s_rf = 1.0 - p.R_w * omega_rf / denom_rf;
        s_lr = 1.0 - p.R_w * omega_lr / denom_lr;
        s_rr = 1.0 - p.R_w * omega_rr / denom_rr;
    }

    // lateral slip angles
    // switch to kinematic model for small velocities
    double alpha_LF, alpha_RF, alpha_LR, alpha_RR;
    if (std::fabs(x[3]) < 0.1) {
        alpha_LF = 0.0;
        alpha_RF = 0.0;
        alpha_LR = 0.0;
        alpha_RR = 0.0;
    } else {
        alpha_LF = std::atan((x[10] + p.a * x[5] - x[14] * (p.R_w - x[16])) /
                             (x[3] + 0.5 * p.T_f * x[5])) - x[2];
        alpha_RF = std::atan((x[10] + p.a * x[5] - x[14] * (p.R_w - x[16])) /
                             (x[3] - 0.5 * p.T_f * x[5])) - x[2];
        alpha_LR = std::atan((x[10] - p.b * x[5] - x[19] * (p.R_w - x[21])) /
                             (x[3] + 0.5 * p.T_r * x[5]));
        alpha_RR = std::atan((x[10] - p.b * x[5] - x[19] * (p.R_w - x[21])) /
                             (x[3] - 0.5 * p.T_r * x[5]));
    }

    // auxiliary suspension movement
    const double z_SLF = (p.h_s - p.R_w + x[16] - x[11]) / std::cos(x[6]) - p.h_s + p.R_w +
                         p.a * x[8] + 0.5 * (x[6] - x[13]) * p.T_f;
    const double z_SRF = (p.h_s - p.R_w + x[16] - x[11]) / std::cos(x[6]) - p.h_s + p.R_w +
                         p.a * x[8] - 0.5 * (x[6] - x[13]) * p.T_f;
    const double z_SLR = (p.h_s - p.R_w + x[21] - x[11]) / std::cos(x[6]) - p.h_s + p.R_w -
                         p.b * x[8] + 0.5 * (x[6] - x[18]) * p.T_r;
    const double z_SRR = (p.h_s - p.R_w + x[21] - x[11]) / std::cos(x[6]) - p.h_s + p.R_w -
                         p.b * x[8] - 0.5 * (x[6] - x[18]) * p.T_r;

    const double dz_SLF = x[17] - x[12] + p.a * x[9] + 0.5 * (x[7] - x[14]) * p.T_f;
    const double dz_SRF = x[17] - x[12] + p.a * x[9] - 0.5 * (x[7] - x[14]) * p.T_f;
    const double dz_SLR = x[22] - x[12] - p.b * x[9] + 0.5 * (x[7] - x[19]) * p.T_r;
    const double dz_SRR = x[22] - x[12] - p.b * x[9] - 0.5 * (x[7] - x[19]) * p.T_r;

    // camber angles
    const double gamma_LF = x[6] + p.D_f * z_SLF + p.E_f * z_SLF * z_SLF;
    const double gamma_RF = x[6] - p.D_f * z_SRF - p.E_f * z_SRF * z_SRF;
    const double gamma_LR = x[6] + p.D_r * z_SLR + p.E_r * z_SLR * z_SLR;
    const double gamma_RR = x[6] - p.D_r * z_SRR - p.E_r * z_SRR * z_SRR;

    // longitudinal tire forces using the magic formula for pure slip
    const double F0_x_LF = utils::formula_longitudinal(s_lf, gamma_LF, F_z_LF, p.tire);
    const double F0_x_RF = utils::formula_longitudinal(s_rf, gamma_RF, F_z_RF, p.tire);
    const double F0_x_LR = utils::formula_longitudinal(s_lr, gamma_LR, F_z_LR, p.tire);
    const double F0_x_RR = utils::formula_longitudinal(s_rr, gamma_RR, F_z_RR, p.tire);

    // lateral tire forces using the magic formula for pure slip
    double F0_y_LF, mu_y_LF;
    {
        auto res = utils::formula_lateral(alpha_LF, gamma_LF, F_z_LF, p.tire);
        F0_y_LF = res.first;
        mu_y_LF = res.second;
    }
    double F0_y_RF, mu_y_RF;
    {
        auto res = utils::formula_lateral(alpha_RF, gamma_RF, F_z_RF, p.tire);
        F0_y_RF = res.first;
        mu_y_RF = res.second;
    }
    double F0_y_LR, mu_y_LR;
    {
        auto res = utils::formula_lateral(alpha_LR, gamma_LR, F_z_LR, p.tire);
        F0_y_LR = res.first;
        mu_y_LR = res.second;
    }
    double F0_y_RR, mu_y_RR;
    {
        auto res = utils::formula_lateral(alpha_RR, gamma_RR, F_z_RR, p.tire);
        F0_y_RR = res.first;
        mu_y_RR = res.second;
    }

    // longitudinal tire forces for combined slip
    const double F_x_LF = utils::formula_longitudinal_comb(s_lf, alpha_LF, F0_x_LF, p.tire);
    const double F_x_RF = utils::formula_longitudinal_comb(s_rf, alpha_RF, F0_x_RF, p.tire);
    const double F_x_LR = utils::formula_longitudinal_comb(s_lr, alpha_LR, F0_x_LR, p.tire);
    const double F_x_RR = utils::formula_longitudinal_comb(s_rr, alpha_RR, F0_x_RR, p.tire);

    // lateral tire forces for combined slip
    const double F_y_LF = utils::formula_lateral_comb(s_lf, alpha_LF, gamma_LF, mu_y_LF, F_z_LF, F0_y_LF, p.tire);
    const double F_y_RF = utils::formula_lateral_comb(s_rf, alpha_RF, gamma_RF, mu_y_RF, F_z_RF, F0_y_RF, p.tire);
    const double F_y_LR = utils::formula_lateral_comb(s_lr, alpha_LR, gamma_LR, mu_y_LR, F_z_LR, F0_y_LR, p.tire);
    const double F_y_RR = utils::formula_lateral_comb(s_rr, alpha_RR, gamma_RR, mu_y_RR, F_z_RR, F0_y_RR, p.tire);

    // auxiliary movements for compliant joint equations
    const double delta_z_f = p.h_s - p.R_w + x[16] - x[11];
    const double delta_z_r = p.h_s - p.R_w + x[21] - x[11];

    const double delta_phi_f = x[6] - x[13];
    const double delta_phi_r = x[6] - x[18];

    const double dot_delta_phi_f = x[7] - x[14];
    const double dot_delta_phi_r = x[7] - x[19];

    const double dot_delta_z_f = x[17] - x[12];
    const double dot_delta_z_r = x[22] - x[12];

    const double dot_delta_y_f = x[10] + p.a * x[5] - x[15];
    const double dot_delta_y_r = x[10] - p.b * x[5] - x[20];

    const double delta_f = delta_z_f * std::sin(x[6]) - x[27] * std::cos(x[6]) -
                           (p.h_raf - p.R_w) * std::sin(delta_phi_f);
    const double delta_r = delta_z_r * std::sin(x[6]) - x[28] * std::cos(x[6]) -
                           (p.h_rar - p.R_w) * std::sin(delta_phi_r);

    const double dot_delta_f = (delta_z_f * std::cos(x[6]) + x[27] * std::sin(x[6])) * x[7] +
                               dot_delta_z_f * std::sin(x[6]) - dot_delta_y_f * std::cos(x[6]) -
                               (p.h_raf - p.R_w) * std::cos(delta_phi_f) * dot_delta_phi_f;
    const double dot_delta_r = (delta_z_r * std::cos(x[6]) + x[28] * std::sin(x[6])) * x[7] +
                               dot_delta_z_r * std::sin(x[6]) - dot_delta_y_r * std::cos(x[6]) -
                               (p.h_rar - p.R_w) * std::cos(delta_phi_r) * dot_delta_phi_r;

    // compliant joint forces
    const double F_RAF = delta_f * p.K_ras + dot_delta_f * p.K_rad;
    const double F_RAR = delta_r * p.K_ras + dot_delta_r * p.K_rad;

    // auxiliary suspension forces (bump stop neglected, squat/lift neglected)
    const double F_SLF = p.m_s * g * p.b / (2.0 * (p.a + p.b)) - z_SLF * p.K_sf - dz_SLF * p.K_sdf +
                         (x[6] - x[13]) * p.K_tsf / p.T_f;
    const double F_SRF = p.m_s * g * p.b / (2.0 * (p.a + p.b)) - z_SRF * p.K_sf - dz_SRF * p.K_sdf -
                         (x[6] - x[13]) * p.K_tsf / p.T_f;
    const double F_SLR = p.m_s * g * p.a / (2.0 * (p.a + p.b)) - z_SLR * p.K_sr - dz_SLR * p.K_sdr +
                         (x[6] - x[18]) * p.K_tsr / p.T_r;
    const double F_SRR = p.m_s * g * p.a / (2.0 * (p.a + p.b)) - z_SRR * p.K_sr - dz_SRR * p.K_sdr -
                         (x[6] - x[18]) * p.K_tsr / p.T_r;

    // auxiliary variables sprung mass
    const double sumX = F_x_LR + F_x_RR +
                        (F_x_LF + F_x_RF) * std::cos(x[2]) -
                        (F_y_LF + F_y_RF) * std::sin(x[2]);

    const double sumN = (F_y_LF + F_y_RF) * p.a * std::cos(x[2]) +
                        (F_x_LF + F_x_RF) * p.a * std::sin(x[2]) +
                        (F_y_RF - F_y_LF) * 0.5 * p.T_f * std::sin(x[2]) +
                        (F_x_LF - F_x_RF) * 0.5 * p.T_f * std::cos(x[2]) +
                        (F_x_LR - F_x_RR) * 0.5 * p.T_r -
                        (F_y_LR + F_y_RR) * p.b;

    const double sumY_s = (F_RAF + F_RAR) * std::cos(x[6]) +
                          (F_SLF + F_SLR + F_SRF + F_SRR) * std::sin(x[6]);

    const double sumL = 0.5 * F_SLF * p.T_f + 0.5 * F_SLR * p.T_r -
                        0.5 * F_SRF * p.T_f - 0.5 * F_SRR * p.T_r
                        - F_RAF / std::cos(x[6]) *
                              (p.h_s - x[11] - p.R_w + x[16] -
                               (p.h_raf - p.R_w) * std::cos(x[13]))
                        - F_RAR / std::cos(x[6]) *
                              (p.h_s - x[11] - p.R_w + x[21] -
                               (p.h_rar - p.R_w) * std::cos(x[18]));

    const double sumZ_s = (F_SLF + F_SLR + F_SRF + F_SRR) * std::cos(x[6]) -
                          (F_RAF + F_RAR) * std::sin(x[6]);

    const double sumM_s = p.a * (F_SLF + F_SRF) - p.b * (F_SLR + F_SRR) +
                          ((F_x_LF + F_x_RF) * std::cos(x[2]) -
                           (F_y_LF + F_y_RF) * std::sin(x[2]) +
                           F_x_LR + F_x_RR) * (p.h_s - x[11]);

    // auxiliary variables unsprung mass (front)
    const double sumL_uf = 0.5 * F_SRF * p.T_f - 0.5 * F_SLF * p.T_f -
                           F_RAF * (p.h_raf - p.R_w)
                           + F_z_LF * (p.R_w * std::sin(x[13]) +
                                       0.5 * p.T_f * std::cos(x[13]) -
                                       p.K_lt * F_y_LF)
                           - F_z_RF * (-p.R_w * std::sin(x[13]) +
                                       0.5 * p.T_f * std::cos(x[13]) +
                                       p.K_lt * F_y_RF)
                           - ((F_y_LF + F_y_RF) * std::cos(x[2]) +
                              (F_x_LF + F_x_RF) * std::sin(x[2])) * (p.R_w - x[16]);

    const double sumL_ur = 0.5 * F_SRR * p.T_r - 0.5 * F_SLR * p.T_r -
                           F_RAR * (p.h_rar - p.R_w)
                           + F_z_LR * (p.R_w * std::sin(x[18]) +
                                       0.5 * p.T_r * std::cos(x[18]) -
                                       p.K_lt * F_y_LR)
                           - F_z_RR * (-p.R_w * std::sin(x[18]) +
                                       0.5 * p.T_r * std::cos(x[18]) +
                                       p.K_lt * F_y_RR)
                           - (F_y_LR + F_y_RR) * (p.R_w - x[21]);

    const double sumZ_uf = F_z_LF + F_z_RF + F_RAF * std::sin(x[6]) -
                           (F_SLF + F_SRF) * std::cos(x[6]);

    const double sumZ_ur = F_z_LR + F_z_RR + F_RAR * std::sin(x[6]) -
                           (F_SLR + F_SRR) * std::cos(x[6]);

    const double sumY_uf = (F_y_LF + F_y_RF) * std::cos(x[2]) +
                           (F_x_LF + F_x_RF) * std::sin(x[2]) -
                           F_RAF * std::cos(x[6]) -
                           (F_SLF + F_SRF) * std::sin(x[6]);

    const double sumY_ur = (F_y_LR + F_y_RR) -
                           F_RAR * std::cos(x[6]) -
                           (F_SLR + F_SRR) * std::sin(x[6]);

    // dynamics common with single-track model
    std::vector<double> f;
    f.reserve(29);

    if (std::fabs(x[3]) < 0.1) {
        // Use kinematic model with reference point at center of mass
        const double lwb = p.a + p.b;

        // system dynamics: kinematic single-track at CoG
        std::array<double, 5> x_ks{
            x[0], x[1], x[2], x[3], x[4]
        };
        std::array<double, 2> u_arr{u[0], u[1]};
        auto f_ks = utils::vehicle_dynamics_ks_cog(x_ks, u_arr, p);

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
    } else {
        // dynamic model
        f.push_back(std::cos(beta + x[4]) * vel);
        f.push_back(std::sin(beta + x[4]) * vel);
        f.push_back(u[0]);
        f.push_back(1.0 / p.m * sumX + x[5] * x[10]);
        f.push_back(x[5]);
        f.push_back(1.0 / (p.I_z - (p.I_xz_s * p.I_xz_s) / p.I_Phi_s) *
                    (sumN + p.I_xz_s / p.I_Phi_s * sumL));
    }

    // remaining sprung mass dynamics
    f.push_back(x[7]);
    f.push_back(1.0 / (p.I_Phi_s - (p.I_xz_s * p.I_xz_s) / p.I_z) *
                (p.I_xz_s / p.I_z * sumN + sumL));
    f.push_back(x[9]);
    f.push_back(1.0 / p.I_y_s * sumM_s);
    f.push_back(1.0 / p.m_s * sumY_s - x[5] * x[3]);
    f.push_back(x[12]);
    f.push_back(g - 1.0 / p.m_s * sumZ_s);

    // unsprung mass dynamics (front)
    f.push_back(x[14]);
    f.push_back(1.0 / p.I_uf * sumL_uf);
    f.push_back(1.0 / p.m_uf * sumY_uf - x[5] * x[3]);
    f.push_back(x[17]);
    f.push_back(g - 1.0 / p.m_uf * sumZ_uf);

    // unsprung mass dynamics (rear)
    f.push_back(x[19]);
    f.push_back(1.0 / p.I_ur * sumL_ur);
    f.push_back(1.0 / p.m_ur * sumY_ur - x[5] * x[3]);
    f.push_back(x[22]);
    f.push_back(g - 1.0 / p.m_ur * sumZ_ur);

    // convert acceleration input to brake and engine torque
    double T_B, T_E;
    if (u[1] > 0.0) {
        T_B = 0.0;
        T_E = p.m * p.R_w * u[1];
    } else {
        T_B = p.m * p.R_w * u[1];
        T_E = 0.0;
    }

    // wheel dynamics (p.T_* new parameters for torque splitting)
    f.push_back(1.0 / p.I_y_w *
                (-p.R_w * F_x_LF + 0.5 * p.T_sb * T_B + 0.5 * p.T_se * T_E));
    f.push_back(1.0 / p.I_y_w *
                (-p.R_w * F_x_RF + 0.5 * p.T_sb * T_B + 0.5 * p.T_se * T_E));
    f.push_back(1.0 / p.I_y_w *
                (-p.R_w * F_x_LR + 0.5 * (1.0 - p.T_sb) * T_B +
                 0.5 * (1.0 - p.T_se) * T_E));
    f.push_back(1.0 / p.I_y_w *
                (-p.R_w * F_x_RR + 0.5 * (1.0 - p.T_sb) * T_B +
                 0.5 * (1.0 - p.T_se) * T_E));

    // negative wheel spin forbidden: if wheel angular speed < 0, zero its derivative
    for (int iState = 23; iState <= 26; ++iState) {
        if (x[iState] < 0.0 && iState < static_cast<int>(f.size())) {
            // Python also sets x[iState] = 0 here, but since x is read-only
            // for the RHS, zeroing f[iState] is the relevant effect.
            f[static_cast<std::size_t>(iState)] = 0.0;
        }
    }

    // compliant joint equations
    f.push_back(dot_delta_y_f);
    f.push_back(dot_delta_y_r);

    return f;
}

} // namespace velox::models
