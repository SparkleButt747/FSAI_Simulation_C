#include "PathConfig.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace {
static double random_double() {
    return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
}
}

PathConfig::PathConfig() {
    seed = random_double();
    minCornerRadius = 3.0;
    maxFrequency = 6;
    amplitude = 1.0 / 3.0;
    checkSelfIntersection = true;
    startingAmplitude = 0.4;
    relativeAccuracy = 0.005;
    margin = 0.0;
    startingStraightLength = 6.0;
    startingStraightDownsample = 2;
    minConeSpacing = 3.0 * M_PI / 16.0;
    maxConeSpacing = 0.6;
    trackWidth = 5.0;
    coneSpacingBias = 1.0;
    startingConeSpacing = 2.5;
    calculateResolutionAndLength();
}

PathConfig::PathConfig(int spacing) {
    seed = random_double();
    minCornerRadius = 3.0;
    maxFrequency = 6;
    amplitude = 1.0 / 3.0;
    checkSelfIntersection = true;
    startingAmplitude = 0.4;
    relativeAccuracy = 0.005;
    margin = 0.0;
    startingStraightLength = 6.0;
    startingStraightDownsample = 2;
    minConeSpacing = spacing;
    maxConeSpacing = spacing;
    trackWidth = 5.0;
    coneSpacingBias = 1.0;
    startingConeSpacing = 2.5;
    calculateResolutionAndLength();
}

void PathConfig::calculateResolutionAndLength() {
    double t = amplitude * maxFrequency;
    double lengthCalc = ((0.6387 * t + 43.86) * t + 123.1) * t + 35.9;
    length = lengthCalc;
    double minSep = minConeSpacing;
    double maxSep = maxConeSpacing;
    double r = std::log(lengthCalc) / std::log(2.0) / minCornerRadius;
    resolution = static_cast<int>(4 * lengthCalc * std::max(1 / minSep, r / maxSep));
}

