#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "detect.hpp"

// Structure for a matched pair of features across stereo frames
struct Feature
{
    double x_1, y_1; // Coordinates in left frame
    double x_2, y_2; // Coordinates in right frame
};

// Structure for an unmatched feature in a single frame
struct PseudoFeature{
    double x, y;
    cv::Mat descriptors;
};

// NEW: Structure to group matched features by their originating cone (bounding box)
struct ConeMatches {
    int cone_index;                 // Index matching the input vector of BoxBounds
    fsai::vision::BoxBound bound;   // The bounding box these matches belong to
    std::vector<Feature> matches;   // The list of stereo matched features for this specific cone
};

// --- Function Declarations ---

// Extracts ROI from a frame based on bounding box
cv::Mat extract_boundimg(const cv::Mat& left_frame, const fsai::vision::BoxBound& box_bound);

// Extracts ORB features from a given image fragment (or full frame)
std::vector<PseudoFeature> extract_features(const cv::Mat& frame, cv::Ptr<cv::ORB> orb);

// Finds stereo matches between two sets of features using epipolar constraint and descriptor matching
std::vector<Feature> pair_features(const std::vector<PseudoFeature>& left_features, const std::vector<PseudoFeature>& right_features);

// Main entry point: matching features for specific objects (cones)
std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame, const cv::Mat& right_frame, const std::vector<fsai::vision::BoxBound>& box_bounds);