#include "models/tire_model.hpp"

#include <cmath>

namespace velox::models {
namespace utils {

double sign(double x)
{
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

double formula_longitudinal(double kappa,
                            double gamma,
                            double F_z,
                            const TireParameters& p)
{
    // turn slip is neglected, so xi_i=1
    // all scaling factors lambda = 1

    // coordinate system transformation
    kappa = -kappa;

    const double S_hx = p.p_hx1;
    const double S_vx = F_z * p.p_vx1;

    const double kappa_x = kappa + S_hx;
    const double mu_x = p.p_dx1 * (1.0 - p.p_dx3 * gamma * gamma);

    const double C_x = p.p_cx1;
    const double D_x = mu_x * F_z;
    const double E_x = p.p_ex1;
    const double K_x = F_z * p.p_kx1;
    const double B_x = K_x / (C_x * D_x);

    // magic tire formula
    return D_x * std::sin(
        C_x * std::atan(B_x * kappa_x -
                        E_x * (B_x * kappa_x - std::atan(B_x * kappa_x))) +
        S_vx
    );
}

std::pair<double, double> formula_lateral(double alpha,
                                          double gamma,
                                          double F_z,
                                          const TireParameters& p)
{
    // turn slip is neglected, so xi_i=1
    // all scaling factors lambda = 1

    const double S_hy = sign(gamma) * (p.p_hy1 + p.p_hy3 * std::fabs(gamma));
    const double S_vy = sign(gamma) * F_z * (p.p_vy1 + p.p_vy3 * std::fabs(gamma));

    const double alpha_y = alpha + S_hy;
    const double mu_y = p.p_dy1 * (1.0 - p.p_dy3 * gamma * gamma);

    const double C_y = p.p_cy1;
    const double D_y = mu_y * F_z;
    const double E_y = p.p_ey1;
    const double K_y = F_z * p.p_ky1; // simplify K_y0 to p.p_ky1*F_z
    const double B_y = K_y / (C_y * D_y);

    const double F_y = D_y * std::sin(
        C_y * std::atan(B_y * alpha_y -
                        E_y * (B_y * alpha_y - std::atan(B_y * alpha_y))) ) +
        S_vy;

    return {F_y, mu_y};
}

double formula_longitudinal_comb(double kappa,
                                 double alpha,
                                 double F0_x,
                                 const TireParameters& p)
{
    // turn slip is neglected, so xi_i=1
    // all scaling factors lambda = 1

    const double S_hxalpha = p.r_hx1;
    const double alpha_s = alpha + S_hxalpha;

    const double B_xalpha = p.r_bx1 * std::cos(std::atan(p.r_bx2 * kappa));
    const double C_xalpha = p.r_cx1;
    const double E_xalpha = p.r_ex1;

    const double denom_angle = C_xalpha * std::atan(
        B_xalpha * S_hxalpha -
        E_xalpha * (B_xalpha * S_hxalpha - std::atan(B_xalpha * S_hxalpha))
    );
    const double D_xalpha = F0_x / std::cos(denom_angle);

    // magic tire formula
    return D_xalpha * std::cos(
        C_xalpha * std::atan(B_xalpha * alpha_s -
                             E_xalpha * (B_xalpha * alpha_s - std::atan(B_xalpha * alpha_s)))
    );
}

double formula_lateral_comb(double kappa,
                            double alpha,
                            double gamma,
                            double mu_y,
                            double F_z,
                            double F0_y,
                            const TireParameters& p)
{
    // turn slip is neglected, so xi_i=1
    // all scaling factors lambda = 1

    const double S_hykappa = p.r_hy1;
    const double kappa_s = kappa + S_hykappa;

    const double B_ykappa = p.r_by1 * std::cos(std::atan(p.r_by2 * (alpha - p.r_by3)));
    const double C_ykappa = p.r_cy1;
    const double E_ykappa = p.r_ey1;

    const double denom_angle = C_ykappa * std::atan(
        B_ykappa * S_hykappa -
        E_ykappa * (B_ykappa * S_hykappa - std::atan(B_ykappa * S_hykappa))
    );
    const double D_ykappa = F0_y / std::cos(denom_angle);

    const double D_vykappa = mu_y * F_z * (p.r_vy1 + p.r_vy3 * gamma) *
                             std::cos(std::atan(p.r_vy4 * alpha));
    const double S_vykappa = D_vykappa * std::sin(p.r_vy5 * std::atan(p.r_vy6 * kappa));

    // magic tire formula
    return D_ykappa * std::cos(
               C_ykappa * std::atan(
                   B_ykappa * kappa_s -
                   E_ykappa * (B_ykappa * kappa_s - std::atan(B_ykappa * kappa_s))
               )
           ) + S_vykappa;
}

} // namespace utils
} // namespace velox::models
