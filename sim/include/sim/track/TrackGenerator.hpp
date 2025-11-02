#pragma once
#include "PathConfig.hpp"
#include "PathGenerator.hpp"
#include "Transform.h"
#include <vector>
#include <complex>

struct TrackResult {
    // Large orange cones marking the start line (two pairs).
    std::vector<Transform> startCones;
    std::vector<Transform> leftCones;
    std::vector<Transform> rightCones;
    std::vector<Transform> checkpoints;
};

class TrackGenerator {
public:
    TrackResult generateTrack(const PathConfig& config, const PathResult& path) const;

private:
    static double complexDistance(const std::complex<double>& a, const std::complex<double>& b);
    static std::vector<double> calcDistanceToNext(const std::vector<std::complex<double>>& points);
    static std::vector<double> rollArray(const std::vector<double>& arr, int shift);
    static std::vector<std::complex<double>> multiplyComplexArrayScalar(const std::vector<std::complex<double>>& arr, double scalar);
    static std::vector<std::complex<double>> addComplexArrays(const std::vector<std::complex<double>>& a, const std::vector<std::complex<double>>& b);
    static std::vector<std::complex<double>> subtractComplexArrays(const std::vector<std::complex<double>>& a, const std::vector<std::complex<double>>& b);
    static std::vector<double> translateDoubleArray(const std::vector<double>& arr, double val);
    static std::vector<std::complex<double>> resampleBoundary(const std::vector<std::complex<double>>& points, int nResample);
    static Vector3 complexToVector3(const std::complex<double>& z);
    static double complexToAngle(const std::complex<double>& z);
    static std::vector<std::complex<double>> placeCones(const std::vector<std::complex<double>>& points,
                                                        const std::vector<double>& radii,
                                                        int side,
                                                        double minConeSpacing,
                                                        double maxConeSpacing,
                                                        double minCornerRadius,
                                                        double trackWidth,
                                                        double coneSpacingBias);
};

