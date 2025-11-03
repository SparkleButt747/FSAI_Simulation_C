#include "centroid.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization> // 引入 LM
#include <iostream>
#include <cmath> // for std::pow

namespace Centroid {

// --- 1. 线性求解器 (最终修正版) ---
Eigen::Vector2d centroid_linear(const std::vector<Eigen::Vector3d>& points) {
    
    int N = points.size();

    // 至少需要 3 个点 (N=3) 来创建 2 个方程
    if (N < 3) {
        return Eigen::Vector2d(0.0, 0.0);
    }

    Eigen::MatrixXd A(N - 1, 2);
    Eigen::VectorXd B(N - 1);

    const auto& p1 = points[0];
    double x1 = p1(0), y1 = p1(1), z1 = p1(2);

    for (int i = 1; i < N; ++i) {
        const auto& pi = points[i];
        double xi = pi(0), yi = pi(1), zi = pi(2);

        // --- 填充 A 矩阵 ---
        double dX = x1 - xi;
        A(i - 1, 0) = 2.0 * dX;
        
        double dY = y1 - yi;
        A(i - 1, 1) = 2.0 * dY;

        // --- 填充 B 向量 (*** 完全数值稳定版 ***) ---
        
        // 1. (x1^2 - xi^2) -> (x1 - xi) * (x1 + xi)
        double stable_x_term = dX * (x1 + xi);

        // 2. (y1^2 - yi^2) -> (y1 - yi) * (y1 + yi)
        double stable_y_term = dY * (y1 + yi);

        // 3. (r1_prime_sq - ri_prime_sq) -> K^2 * (zi - z1) * (2*H - z1 - zi)
        double dZ_inv = zi - z1; // (zi - z1)
        double z_sum = z1 + zi;
        double stable_r_term = std::pow(K, 2) * dZ_inv * (2.0 * CONE_H - z_sum);

        // B(i-1) = (x_term) + (y_term) - (r_term)
        B(i - 1) = stable_x_term + stable_y_term - stable_r_term;
    }

    // 使用 SVD 求解 Ax = B
    Eigen::VectorXd sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
    
    return Eigen::Vector2d(sol(0), sol(1));
}


// --- 2. 非线性求解器 (Functor, 这是我们之前修复好的) ---
struct ConeResidualsFunctor {
    const std::vector<Eigen::Vector3d>& m_points;
    int m_num_points;

    ConeResidualsFunctor(const std::vector<Eigen::Vector3d>& points)
        : m_points(points), m_num_points(points.size()) {}

    // (a) 计算残差 fvec
    int operator()(const Eigen::VectorXd& center, Eigen::VectorXd& fvec) const {
        double cx = center(0);
        double cy = center(1);

        for (int i = 0; i < m_num_points; ++i) {
            double xi = m_points[i](0);
            double yi = m_points[i](1);
            double zi = m_points[i](2);

            double dist_observed = std::sqrt(std::pow(xi - cx, 2) + std::pow(yi - cy, 2));
            double r_theoretical = K * (CONE_H - zi);
            
            fvec(i) = dist_observed - r_theoretical;
        }
        return 0;
    }

    // (b) 计算雅可比矩阵 jac (确保这个也是对的, 尤其是负号)
    int df(const Eigen::VectorXd& center, Eigen::MatrixXd& jac) const {
        double cx = center(0);
        double cy = center(1);

        for (int i = 0; i < m_num_points; ++i) {
            double xi = m_points[i](0);
            double yi = m_points[i](1);

            double dx = xi - cx;
            double dy = yi - cy;
            double dist_observed = std::sqrt(dx * dx + dy * dy);

            if (dist_observed < 1e-6) {
                jac(i, 0) = 0;
                jac(i, 1) = 0;
            } else {
                jac(i, 0) = -dx / dist_observed; // 负号
                jac(i, 1) = -dy / dist_observed; // 负号
            }
        }
        return 0;
    }

    // (c) 告诉 LM 求解器尺寸
    int inputs() const { return 2; } 
    int values() const { return m_num_points; } 
};


// --- 3. 非线性求解器 (调用) ---
Eigen::Vector2d centroid_nonlinear(const std::vector<Eigen::Vector3d>& points,
                                     const Eigen::Vector2d& initial_guess) {
    if (points.empty()) {
        return initial_guess;
    }

    ConeResidualsFunctor functor(points);
    Eigen::LevenbergMarquardt<ConeResidualsFunctor> lm(functor);
    
    Eigen::VectorXd params(2);
    params(0) = initial_guess(0);
    params(1) = initial_guess(1);

    lm.minimize(params);

    return params;
}

} // namespace Centroid