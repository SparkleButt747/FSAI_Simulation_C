#include <iostream>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "features.hpp"

// --- Assuming these structs exist based on your code ---

cv::Mat extract_boundimg(const cv::Mat& left_frame, const fsai::vision::BoxBound& box_bound) {
    cv::Rect roi(box_bound.x, box_bound.y, box_bound.w, box_bound.h);
    roi &= cv::Rect(0, 0, left_frame.cols, left_frame.rows);

    if (roi.width <= 0 || roi.height <= 0) {
        std::cerr << "Warning: Out of bounds ROI requested." << std::endl;
        return cv::Mat();
    }
    return left_frame(roi);
}

std::vector<PseudoFeature> extract_features(const cv::Mat& frame, cv::Ptr<cv::ORB> orb) {
    if (frame.empty()) return {};

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb->detectAndCompute(frame, cv::noArray(), keypoints, descriptors); // More efficient combined call

    std::vector<PseudoFeature> results;
    results.reserve(keypoints.size());

    for (size_t i = 0; i < keypoints.size(); ++i) {
        results.push_back({(double)keypoints[i].pt.x, (double)keypoints[i].pt.y, descriptors.row(i)});
    }
    return results;
}

std::vector<Feature> pair_features(const std::vector<PseudoFeature>& left_features, const std::vector<PseudoFeature>& right_features) {
    std::vector<Feature> results;
    if (left_features.empty() || right_features.empty()) return results;

    // 1. Build Y-hash for right Features for quick epipolar lookup
    std::unordered_map<int, std::vector<int>> y_mappings; // Store INDICES, not copies of full Features
    for (size_t i = 0; i < right_features.size(); ++i) {
        y_mappings[(int)std::round(right_features[i].y)].push_back(i);
    }

    int epipolar_tolerance = 1;
    int max_hamming_dist = 50; // THRESHOLD: Discard bad matches (typical ORB val is 30-70)

    for (const auto& left_feat : left_features) {
        int y_start = std::max(0, (int)std::round(left_feat.y) - epipolar_tolerance);
        int y_end = (int)std::round(left_feat.y) + epipolar_tolerance;

        int best_idx = -1;
        int min_dist = max_hamming_dist + 1; // Init with threshold + 1

        for (int y = y_start; y <= y_end; ++y) {
            auto it = y_mappings.find(y);
            if (it != y_mappings.end()) {
                for (int right_idx : it->second) {
                    // Optimization: Only match if right Feature is to the LEFT of matched left Feature 
                    // (standard stereo assumption: disparity is positive, so x_left >= x_right)
                    if (right_features[right_idx].x > left_feat.x) continue; 

                     int dist = cv::norm(left_feat.descriptors, right_features[right_idx].descriptors, cv::NORM_HAMMING);
                     if (dist < min_dist) {
                         min_dist = dist;
                         best_idx = right_idx;
                     }
                }
            }
        }

        if (best_idx != -1) {
            results.push_back({left_feat.x, left_feat.y, right_features[best_idx].x, right_features[best_idx].y});
        }
    }
    return results;
}

// --- CHANGED RETURN TYPE ---
std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame, const cv::Mat& right_frame, const std::vector<fsai::vision::BoxBound>& box_bounds) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    
    // 1. Extract ALL Features from the right frame once (expensive operation)
    std::vector<PseudoFeature> right_features = extract_features(right_frame, orb);
    
    std::vector<ConeMatches> all_cone_matches;
    all_cone_matches.reserve(box_bounds.size());

    // 2. Process each cone individually
    for (size_t i = 0; i < box_bounds.size(); ++i) {
        cv::Mat cone_img = extract_boundimg(left_frame, box_bounds[i]);
        if (cone_img.empty()) continue;

        std::vector<PseudoFeature> cone_left_features = extract_features(cone_img, orb);

        // 3. Adjust coordinates from ROI-relative to full-frame-relative
        for (auto& feat : cone_left_features) {
            feat.x += box_bounds[i].x;
            feat.y += box_bounds[i].y;
        }

        // 4. Match THIS cone's Features against ALL right Features
        //    (It's okay to match against all right Features because epipolar 
        //     and descriptor constraints will find the correct corresponding region in the right image automatically)
        std::vector<Feature> matches = pair_features(cone_left_features, right_features);

        if (!matches.empty()) {
             all_cone_matches.push_back({(int)i, box_bounds[i], matches});
        }
    }

    return all_cone_matches;
}