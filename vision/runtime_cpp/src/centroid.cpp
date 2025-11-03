#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <vector>
#include <iostream>

#include "centroid.hpp"

namespace Centroid {

// Linear Approach
Eigen::Vector2d centroid_linear(const std::vector<Eigen::Vector3d>& points) {
    
    int N = points.size();

    // At least 3 points are needed
    if (N < 3) {
        return Eigen::Vector2d(0.0, 0.0);
    }

    // Matrix A
    Eigen::MatrixXd A(N - 1, 2);
    // Vector B
    Eigen::VectorXd B(N - 1);

    const auto& p1 = points[0];
    double x1 = p1(0), y1 = p1(1), z1 = p1(2);
    double r1_prime_sq = std::pow(K * (CONE_H - z1), 2);
    double c1 = x1 * x1 + y1 * y1 - r1_prime_sq;

    for (int i = 1; i < N; ++i) {
        const auto& pi = points[i];
        double xi = pi(0), yi = pi(1), zi = pi(2);
        double ri_prime_sq = std::pow(K * (CONE_H - zi), 2);
        double ci = xi * xi + yi * yi - ri_prime_sq;

        A(i - 1, 0) = 2.0 * (x1 - xi);
        A(i - 1, 1) = 2.0 * (y1 - yi);
        B(i - 1) = c1 - ci;
    }

    // Use SVD to solve Ax = B, which is equivalent to np.linalg.lstsq in Python
    Eigen::VectorXd sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
    
    if (sol.size() == 2) {
        return Eigen::Vector2d(sol(0), sol(1));
    } else {
        return Eigen::Vector2d(0.0, 0.0);
    }
}


// Non-Linear Approach
struct ConeResidualsFunctor : Eigen::DenseFunctor<double> {
    const std::vector<Eigen::Vector3d>& m_points;
    int m_num_points;

    // Constructor
    ConeResidualsFunctor(const std::vector<Eigen::Vector3d>& points)
        : Eigen::DenseFunctor<double>(2, static_cast<int>(points.size())),
        m_points(points),
        m_num_points(static_cast<int>(points.size()))
    {}

    // Calculate residuals
    int operator()(const Eigen::VectorXd& center, Eigen::VectorXd& fvec) const {
        if (center.size() < 2 || fvec.size() != m_num_points) {
            return -1; // Error
        }

        double cx = center(0);
        double cy = center(1);

        for (int i = 0; i < m_num_points; ++i) {
            double xi = m_points[i](0);
            double yi = m_points[i](1);
            double zi = m_points[i](2);

            double distance = std::sqrt(std::pow(xi - cx, 2) + std::pow(yi - cy, 2));
            
            double r_theory = K * (CONE_H - zi);

            fvec(i) = distance - r_theory;
        }
        // Success
        return 0;
    }

    // Number of inputs
    int inputs() const { return 2; }
    // Number of residuals
    int values() const { return m_num_points; }
};


Eigen::Vector2d centroid_nonlinear(const std::vector<Eigen::Vector3d>& points,
                                     const Eigen::Vector2d& initial_guess) {
    if (points.empty()) {
        return initial_guess;
    }

    // Create functor
    ConeResidualsFunctor functor(points);

    // Prepare LM solver
    Eigen::LevenbergMarquardt<ConeResidualsFunctor> lm(functor);
    
    // Prepare initial parameters
    Eigen::VectorXd params(2);
    params(0) = initial_guess(0);
    params(1) = initial_guess(1);

    int _ = lm.minimize(params);

    return Eigen::Vector2d(params(0), params(1));
}

}