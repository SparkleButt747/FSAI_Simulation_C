#include "TrackGenerator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
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

namespace {

std::vector<double> downsampleDistances(const std::vector<cdouble>& points) {
    const int count = static_cast<int>(points.size());
    std::vector<double> distances(count, 0.0);
    if (count == 0) {
        return distances;
    }
    for (int i = 0; i < count; ++i) {
        const int next = (i + 1) % count;
        distances[i] = std::abs(points[next] - points[i]);
    }
    return distances;
}

double computeTrackLength(const std::vector<cdouble>& positions) {
    double length = 0.0;
    const int count = static_cast<int>(positions.size());
    for (int i = 0; i < count; ++i) {
        const int next = (i + 1) % count;
        length += std::abs(positions[next] - positions[i]);
    }
    return length;
}

}  // namespace

void TrackGenerator::pickStartingPoint(std::vector<cdouble>& positions,
                                       std::vector<cdouble>& normals,
                                       std::vector<double>& radii,
                                       double startingStraightLength,
                                       int downsample) {
    const int nPoints = static_cast<int>(positions.size());
    if (nPoints == 0) {
        return;
    }

    if (downsample < 1) {
        downsample = 1;
    }

    std::vector<cdouble> dsPositions;
    std::vector<double> dsCurvature;
    dsPositions.reserve(nPoints / downsample + 1);
    dsCurvature.reserve(nPoints / downsample + 1);

    for (int i = 0; i < nPoints; i += downsample) {
        dsPositions.push_back(positions[i]);
        const double radius = radii[i];
        double curvature = 0.0;
        if (radius != 0.0) {
            curvature = std::abs(1.0 / radius);
        } else {
            curvature = std::numeric_limits<double>::infinity();
        }
        dsCurvature.push_back(curvature);
    }

    const int dsCount = static_cast<int>(dsPositions.size());
    if (dsCount == 0) {
        return;
    }

    std::vector<int> sortedIndices(dsCount);
    std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
    std::sort(sortedIndices.begin(), sortedIndices.end(),
              [&](int a, int b) { return dsCurvature[a] < dsCurvature[b]; });

    const int candidateCount = std::max(1, dsCount / 8);
    const double smoothDiameter = 1.5 * startingStraightLength;
    const std::vector<double> dsDistances = downsampleDistances(dsPositions);

    auto smoothedCurvature = [&](int index) {
        double value = dsCurvature[index];
        double coefSum = 1.0;
        if (dsCount <= 1) {
            return value;
        }
        int current = (index == 0) ? dsCount - 1 : index - 1;
        double distance = dsDistances[current];
        while (distance < smoothDiameter && dsCount > 1) {
            const double coef = dsDistances[current] *
                                std::sin(M_PI * distance / smoothDiameter);
            value += coef * dsCurvature[current];
            coefSum += coef;
            current = (current == 0) ? dsCount - 1 : current - 1;
            distance += dsDistances[current];
        }
        return coefSum > 0.0 ? value / coefSum : value;
    };

    double bestValue = std::numeric_limits<double>::infinity();
    int bestIndex = 0;
    for (int i = 0; i < candidateCount && i < dsCount; ++i) {
        const int candidate = sortedIndices[i];
        const double smoothed = smoothedCurvature(candidate);
        if (smoothed < bestValue) {
            bestValue = smoothed;
            bestIndex = candidate;
        }
    }

    const int startIndex = (bestIndex * downsample) % nPoints;

    auto rotateVector = [&](auto& vec) {
        std::rotate(vec.begin(), vec.begin() + startIndex, vec.end());
    };
    rotateVector(positions);
    rotateVector(normals);
    rotateVector(radii);

    const cdouble startPosition = positions.front();
    for (auto& p : positions) {
        p -= startPosition;
    }

    const cdouble startNormal = normals.front();
    cdouble rotation = cdouble(0.0, 1.0);
    if (std::abs(startNormal) > 0.0) {
        rotation = cdouble(0.0, 1.0) / startNormal;
    }
    for (auto& p : positions) {
        p *= rotation;
    }
    for (auto& n : normals) {
        n *= rotation;
    }
}

int TrackGenerator::findCarIndex(const std::vector<cdouble>& positions,
                                 double startOffset) {
    const int nPoints = static_cast<int>(positions.size());
    if (nPoints == 0) {
        return 0;
    }

    const double totalLength = computeTrackLength(positions);
    const double targetOffset = std::min(startOffset, totalLength);

    int current = 0;
    double accumulated = 0.0;
    std::vector<bool> visited(static_cast<std::size_t>(nPoints), false);

    while (accumulated < targetOffset) {
        const int prev = (current - 1 + nPoints) % nPoints;
        accumulated += std::abs(positions[current] - positions[prev]);
        current = prev;
        if (visited[static_cast<std::size_t>(current)]) {
            break;
        }
        visited[static_cast<std::size_t>(current)] = true;
        if (accumulated >= totalLength) {
            break;
        }
    }
    return current;
}

void TrackGenerator::applyTransform(std::vector<cdouble>& points,
                                    const cdouble& translation,
                                    const cdouble& rotation) {
    for (auto& point : points) {
        point = (point - translation) * rotation;
    }
}

std::vector<cdouble> TrackGenerator::placeCones(const std::vector<cdouble>& points,
                                                const std::vector<double>& radii,
                                                int side,
                                                double minConeSpacing,
                                                double maxConeSpacing,
                                                double minCornerRadius,
                                                double trackWidth,
                                                double coneSpacingBias) {
    int nPoints = static_cast<int>(points.size());
    if (nPoints == 0) {
        return {};
    }

    double minDensity = 1.0 / maxConeSpacing;
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
    for (double d : coneDensity) {
        modifiedLength += d;
    }
    double rounded = std::round(modifiedLength);
    if (rounded <= 0.0) {
        rounded = 1.0;
    }
    double threshold = modifiedLength / rounded;

    std::vector<cdouble> selectedPoints;
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
    return selectedPoints;
}

TrackResult TrackGenerator::generateTrack(const PathConfig& config, const PathResult& path) const {
    int nPoints = static_cast<int>(path.points.size());
    double trackWidth = config.trackWidth;
    double minConeSpacing = config.minConeSpacing;
    double maxConeSpacing = config.maxConeSpacing;
    double coneSpacingBias = config.coneSpacingBias;
    double minCornerRadius = config.minCornerRadius;

    std::vector<cdouble> pointsC(nPoints);
    std::vector<cdouble> normalsC(nPoints);
    for (int i = 0; i < nPoints; ++i) {
        pointsC[i] = cdouble(path.points[i].x(), path.points[i].y());
        normalsC[i] = cdouble(path.normals[i].x(), path.normals[i].y());
    }

    std::vector<double> radii(path.cornerRadii.begin(), path.cornerRadii.end());
    pickStartingPoint(pointsC, normalsC, radii, config.startingStraightLength,
                      config.startingStraightDownsample);

    auto leftPoints = addComplexArrays(pointsC, multiplyComplexArrayScalar(normalsC, trackWidth / 2.0));
    auto rightPoints = subtractComplexArrays(pointsC, multiplyComplexArrayScalar(normalsC, trackWidth / 2.0));

    auto leftRadii = translateDoubleArray(radii, -trackWidth / 2.0);
    auto rightRadii = translateDoubleArray(radii, trackWidth / 2.0);

    std::vector<cdouble> leftConesC = placeCones(leftPoints, leftRadii, 1, minConeSpacing, maxConeSpacing,
                                                minCornerRadius, trackWidth, coneSpacingBias);
    std::vector<cdouble> rightConesC = placeCones(rightPoints, rightRadii, -1, minConeSpacing, maxConeSpacing,
                                                 minCornerRadius, trackWidth, coneSpacingBias);

    std::vector<cdouble> startConesC;
    if (!leftConesC.empty() && !rightConesC.empty()) {
        const cdouble offset(config.startingConeSpacing / 2.0, 0.0);
        const cdouble leftStart = leftConesC.front();
        const cdouble rightStart = rightConesC.front();
        startConesC.push_back(leftStart + offset);
        startConesC.push_back(rightStart + offset);
        startConesC.push_back(leftStart - offset);
        startConesC.push_back(rightStart - offset);
    }

    const int carIndex = findCarIndex(pointsC, config.startingStraightLength);
    const cdouble carTranslation = pointsC[carIndex];
    cdouble carNormal = normalsC[carIndex];
    if (std::abs(carNormal) == 0.0) {
        carNormal = cdouble(0.0, 1.0);
    }
    const cdouble carRotation = cdouble(0.0, 1.0) / carNormal;

    applyTransform(leftConesC, carTranslation, carRotation);
    applyTransform(rightConesC, carTranslation, carRotation);
    applyTransform(startConesC, carTranslation, carRotation);

    if (!leftConesC.empty()) {
        leftConesC.erase(leftConesC.begin());
    }
    if (!rightConesC.empty()) {
        rightConesC.erase(rightConesC.begin());
    }

    const double minStartClearance = std::max(config.startingConeSpacing, config.trackWidth);
    if (leftConesC.size() > 1 && std::abs(leftConesC.front()) < minStartClearance) {
        leftConesC.erase(leftConesC.begin());
    }
    if (rightConesC.size() > 1 && std::abs(rightConesC.front()) < minStartClearance) {
        rightConesC.erase(rightConesC.begin());
    }

    if (leftConesC.size() > 1 && rightConesC.size() > 1) {
        const cdouble center0 = 0.5 * (leftConesC[0] + rightConesC[0]);
        const cdouble center1 = 0.5 * (leftConesC[1] + rightConesC[1]);
        const cdouble forward = center1 - center0;
        const cdouble to_left = leftConesC[0] - center0;
        const double cross = forward.real() * to_left.imag() - forward.imag() * to_left.real();
        if (cross < 0.0) {
            leftConesC.swap(rightConesC);
        }
    }

    int nResample = static_cast<int>(std::min(leftConesC.size(), rightConesC.size()));
    if (nResample < 2) throw std::runtime_error("Not enough cones on one side to compute a racing line.");

    auto leftResampled = resampleBoundary(leftConesC, nResample);
    auto rightResampled = resampleBoundary(rightConesC, nResample);

    std::vector<Transform> leftTransforms(nResample);
    std::vector<Transform> rightTransforms(nResample);
    std::vector<Transform> checkpoints(nResample);
    std::vector<Transform> startTransforms(startConesC.size());

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
    for (int i = 0; i < static_cast<int>(startConesC.size()); ++i) {
        startTransforms[i].position = complexToVector3(startConesC[i]);
        startTransforms[i].yaw = 0.0f;
    }

    TrackResult result;
    result.startCones = std::move(startTransforms);
    result.leftCones = std::move(leftTransforms);
    result.rightCones = std::move(rightTransforms);
    result.checkpoints = std::move(checkpoints);
    return result;
}

