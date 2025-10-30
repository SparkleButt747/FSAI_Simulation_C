#pragma once
#include <sim/track/PathConfig.hpp>
#include <vector>
#include <complex>
#include <Eigen/Core>

struct PathResult {
    std::vector<Eigen::Vector2f> points;
    std::vector<Eigen::Vector2f> normals;
    std::vector<float> cornerRadii;
};

class PathGenerator {
public:
    explicit PathGenerator(const PathConfig& cfg) : config(cfg) {}

    PathResult generatePath(int nPoints) const;

private:
    const PathConfig& config;
    std::vector<double> computeCornerRadii(double dt, const std::vector<std::complex<double>>& dPdt) const;
};

