#pragma once

namespace velox::models {
namespace utils {

/**
 * TireParameters
 * Class defines all Tire Parameters (Pacejka-type coefficients).
 */
struct TireParameters {
    // longitudinal coefficients
    double p_cx1{}; // Shape factor Cfx for longitudinal force
    double p_dx1{}; // Longitudinal friction Mux at Fznom
    double p_dx3{}; // Variation of friction Mux with camber
    double p_ex1{}; // Longitudinal curvature Efx at Fznom
    double p_kx1{}; // Longitudinal slip stiffness Kfx/Fz at Fznom
    double p_hx1{}; // Horizontal shift Shx at Fznom
    double p_vx1{}; // Vertical shift Svx/Fz at Fznom
    double r_bx1{}; // Slope factor for combined slip Fx reduction
    double r_bx2{}; // Variation of slope Fx reduction with kappa
    double r_cx1{}; // Shape factor for combined slip Fx reduction
    double r_ex1{}; // Curvature factor of combined Fx
    double r_hx1{}; // Shift factor for combined slip Fx reduction

    // lateral coefficients
    double p_cy1{}; // Shape factor Cfy for lateral forces
    double p_dy1{}; // Lateral friction Muy
    double p_dy3{}; // Variation of friction Muy with squared camber
    double p_ey1{}; // Lateral curvature Efy at Fznom
    double p_ky1{}; // Maximum value of stiffness Kfy/Fznom
    double p_hy1{}; // Horizontal shift Shy at Fznom
    double p_hy3{}; // Variation of shift Shy with camber
    double p_vy1{}; // Vertical shift in Svy/Fz at Fznom
    double p_vy3{}; // Variation of shift Svy/Fz with camber
    double r_by1{}; // Slope factor for combined Fy reduction
    double r_by2{}; // Variation of slope Fy reduction with alpha
    double r_by3{}; // Shift term for alpha in slope Fy reduction
    double r_cy1{}; // Shape factor for combined Fy reduction
    double r_ey1{}; // Curvature factor of combined Fy
    double r_hy1{}; // Shift factor for combined Fy reduction
    double r_vy1{}; // Kappa induced side force Svyk/Muy*Fz at Fznom
    double r_vy3{}; // Variation of Svyk/Muy*Fz with camber
    double r_vy4{}; // Variation of Svyk/Muy*Fz with alpha
    double r_vy5{}; // Variation of Svyk/Muy*Fz with kappa
    double r_vy6{}; // Variation of Svyk/Muy*Fz with atan(kappa)
};

} // namespace utils
} // namespace velox::models
