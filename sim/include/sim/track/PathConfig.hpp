#pragma once

class PathConfig {
public:
    double seed;
    double minCornerRadius;
    int maxFrequency;
    double amplitude;
    bool checkSelfIntersection;
    double startingAmplitude;
    double relativeAccuracy;
    double margin;
    double startingStraightLength;
    int startingStraightDownsample;
    double minConeSpacing;
    double maxConeSpacing;
    double trackWidth;
    double coneSpacingBias;
    double startingConeSpacing;
    int resolution;
    double length;

    PathConfig();

    PathConfig(int spacing);

    void calculateResolutionAndLength();
};

