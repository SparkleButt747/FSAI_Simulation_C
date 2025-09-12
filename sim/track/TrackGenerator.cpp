#include "TrackGenerator.hpp"
#include <cmath>
#include <stdexcept>

using cdouble = std::complex<double>;

double TrackGenerator::complexDistance(const cdouble& a, const cdouble& b) {
    return std::abs(a - b);
}

std::vector<double> TrackGenerator::calcDistanceToNext(const std::vector<cdouble>& points) {
    int n = points.size();
    std::vector<double> distances(n);
    for (int i = 0; i < n - 1; ++i) {
        distances[i] = complexDistance(points[i+1], points[i]);
    };
    distances[n-1] = complexDistance(points[0], points[n-1]);
    return distances;
}

std::vector<double> TrackGenerator::rollArray(const std::vector<double>& arr, int shift) {
    int n = arr.size();
    std::vector<double> rolled(n);
    for (int i = 0; i < n; ++i) {
        int newIndex = (i - shift + n) % n;
        rolled[newIndex] = arr[i];
    }
    return rolled;
}

std::vector<cdouble> TrackGenerator::multiplyComplexArrayScalar(const std::vector<cdouble>& arr, double scalar) {
    int n = arr.size();
    std::vector<cdouble> result(n);
    for (int i = 0; i < n; ++i) result[i] = arr[i] * scalar;
    return result;
}

std::vector<cdouble> TrackGenerator::addComplexArrays(const std::vector<cdouble>& a, const std::vector<cdouble>& b) {
    int n = a.size();
    std::vector<cdouble> result(n);
    for (int i = 0; i < n; ++i) result[i] = a[i] + b[i];
    return result;
}

std::vector<cdouble> TrackGenerator::subtractComplexArrays(const std::vector<cdouble>& a, const std::vector<cdouble>& b) {
    int n = a.size();
    std::vector<cdouble> result(n);
    for (int i = 0; i < n; ++i) result[i] = a[i] - b[i];
    return result;
}

std::vector<double> TrackGenerator::translateDoubleArray(const std::vector<double>& arr, double val) {
    int n = arr.size();
    std::vector<double> result(n);
    for (int i = 0; i < n; ++i) result[i] = arr[i] + val;
    return result;
}

std::vector<cdouble> TrackGenerator::resampleBoundary(const std::vector<cdouble>& points, int nResample) {
    int nPoints = points.size();
    if (nPoints < 2 || nResample < 1) return {};
    std::vector<double> cum(nPoints);
    cum[0] = 0.0;
    for (int i = 1; i < nPoints; ++i) {
        cum[i] = cum[i-1] + std::abs(points[i] - points[i-1]);
    }
    double total = cum[nPoints-1];
    std::vector<cdouble> resampled(nResample);
    for (int j = 0; j < nResample; ++j) {
        double target = (nResample == 1) ? 0.0 : (j * total / (nResample - 1));
        int i = 0;
        while (i < nPoints - 1 && cum[i+1] < target) ++i;
        double segment = cum[i+1] - cum[i];
        double fraction = (segment > 0) ? (target - cum[i]) / segment : 0.0;
        resampled[j] = points[i] + fraction * (points[i+1] - points[i]);
    }
    return resampled;
}

Vector3 TrackGenerator::complexToVector3(const cdouble& z) {
    Vector3 v; v.x = static_cast<float>(z.real()); v.y = 0.0f; v.z = static_cast<float>(z.imag()); return v;
}

double TrackGenerator::complexToAngle(const cdouble& z) {
    return std::atan2(z.imag(), z.real());
}

int TrackGenerator::placeCones(const std::vector<cdouble>& points, const std::vector<double>& radii, int side,
                               double minConeSpacing, double maxConeSpacing, double minCornerRadius, double trackWidth,
                               double coneSpacingBias, std::vector<cdouble>& selectedPoints) {
    int nPoints = points.size();
    double minDensity = (1.0 / maxConeSpacing) + 0.1;
    double maxDensity = 1.0 / minConeSpacing;
    double densityRange = maxDensity - minDensity;
    double c1 = densityRange / 2.0 * ((1 - coneSpacingBias) * minCornerRadius - (1 + coneSpacingBias) * trackWidth / 2.0);
    double c2 = densityRange / 2.0 * ((1 + coneSpacingBias) * minCornerRadius - (1 - coneSpacingBias) * trackWidth / 2.0);

    std::vector<double> distToNext = calcDistanceToNext(points);
    std::vector<double> distToPrev = rollArray(distToNext, 1);
    std::vector<double> coneDensity(nPoints);
    for (int i = 0; i < nPoints; ++i) {
        coneDensity[i] = minDensity + (side * c1 / radii[i]) + (c2 / std::fabs(radii[i]));
        coneDensity[i] *= distToPrev[i];
    }
    double modifiedLength = 0.0;
    for (double d : coneDensity) modifiedLength += d;
    int rounded = static_cast<int>(std::round(modifiedLength));
    double threshold = (rounded != 0) ? modifiedLength / rounded : 1.0;

    selectedPoints.clear();
    selectedPoints.reserve(nPoints);
    selectedPoints.push_back(points[0]);
    double current = 0.0;
    for (int i = 1; i < nPoints; ++i) {
        current += coneDensity[i];
        if (current >= threshold) {
            current -= threshold;
            selectedPoints.push_back(points[i]);
        }
    }
    return selectedPoints.size();
}

TrackResult TrackGenerator::generateTrack(const PathConfig& config, const PathResult& path) const {
    int nPoints = static_cast<int>(path.points.size());
    double trackWidth = config.trackWidth;
    double minConeSpacing = config.minConeSpacing;
    double maxConeSpacing = config.maxConeSpacing;
    double coneSpacingBias = config.coneSpacingBias;
    double minCornerRadius = config.minCornerRadius;

    std::vector<cdouble> pointsC(nPoints); std::vector<cdouble> normalsC(nPoints);
    for (int i = 0; i < nPoints; ++i) {
        pointsC[i] = cdouble(path.points[i].x(), path.points[i].y());
        normalsC[i] = cdouble(path.normals[i].x(), path.normals[i].y());
    }

    auto leftPoints = addComplexArrays(pointsC, multiplyComplexArrayScalar(normalsC, trackWidth / 2.0));
    auto rightPoints = subtractComplexArrays(pointsC, multiplyComplexArrayScalar(normalsC, trackWidth / 2.0));

    std::vector<double> radii(path.cornerRadii.begin(), path.cornerRadii.end());
    auto leftRadii = translateDoubleArray(radii, -trackWidth / 2.0);
    auto rightRadii = translateDoubleArray(radii, trackWidth / 2.0);

    std::vector<cdouble> leftConesC, rightConesC;
    int nLeftCones = placeCones(leftPoints, leftRadii, 1, minConeSpacing, maxConeSpacing, minCornerRadius, trackWidth, coneSpacingBias, leftConesC);
    int nRightCones = placeCones(rightPoints, rightRadii, -1, minConeSpacing, maxConeSpacing, minCornerRadius, trackWidth, coneSpacingBias, rightConesC);

    int nResample = std::min(nLeftCones, nRightCones);
    if (nResample < 2) throw std::runtime_error("Not enough cones on one side to compute a racing line.");

    auto leftResampled = resampleBoundary(leftConesC, nResample);
    auto rightResampled = resampleBoundary(rightConesC, nResample);

    std::vector<Transform> leftTransforms(nResample);
    std::vector<Transform> rightTransforms(nResample);
    std::vector<Transform> checkpoints(nResample);

    for (int i = 0; i < nResample; ++i) {
        leftTransforms[i].position = complexToVector3(leftResampled[i]);
        leftTransforms[i].yaw = 0.0f;
        rightTransforms[i].position = complexToVector3(rightResampled[i]);
        rightTransforms[i].yaw = 0.0f;
        double cx = (leftResampled[i].real() + rightResampled[i].real()) / 2.0;
        double cz = (leftResampled[i].imag() + rightResampled[i].imag()) / 2.0;
        checkpoints[i].position.x = static_cast<float>(cx);
        checkpoints[i].position.y = 0.0f;
        checkpoints[i].position.z = static_cast<float>(cz);
        double dx = rightResampled[i].real() - leftResampled[i].real();
        double dz = rightResampled[i].imag() - leftResampled[i].imag();
        checkpoints[i].yaw = static_cast<float>(std::atan2(dz, dx));
    }

    TrackResult result;
    result.leftCones = std::move(leftTransforms);
    result.rightCones = std::move(rightTransforms);
    result.checkpoints = std::move(checkpoints);
    return result;
}

