#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "detect.hpp"

// Structure for a matched pair of features across stereo frames
struct Feature
{
    int cone_index;
    double x_1, y_1; // Coordinates in left frame
    double x_2, y_2; // Coordinates in right frame
};

// Structure for an unmatched feature in a single frame
struct PseudoFeature{
    int cone_index;
    double x, y;
    cv::Mat descriptor;
};

// NEW: Structure to group matched features by their originating cone (bounding box)
struct ConeMatches {
    int cone_index;                 // Index matching the input vector of BoxBounds
    fsai::vision::BoxBound bound;   // The bounding box these matches belong to
    std::vector<Feature> matches;   // The list of stereo matched features for this specific cone
};

// --- Function Declarations ---

// Main entry point: matching features for specific objects (cones)
std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame, const cv::Mat& right_frame, const std::vector<fsai::vision::BoxBound>& box_bounds);
