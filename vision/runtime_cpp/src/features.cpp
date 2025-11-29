#include "features.hpp"
#include "common/include/common/types.h"
#include "detect.hpp"

#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include <opencv2/features2d.hpp>  // For SIFT and other feature detectors
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// TUNING: Epipolar search margin (pixels up/down to look in right image)
static const int kEpipolarMargin = 2;

bool is_safe_roi(const cv::Mat& frame, cv::Rect roi) {
    return (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= frame.cols &&
            roi.y + roi.height <= frame.rows && roi.width > 0 && roi.height > 0);
}

float L2_score(cv::Mat left_descriptor, cv::Mat right_descriptor) {
    float dist = cv::norm(left_descriptor, right_descriptor, cv::NORM_L2);
    return dist;
}

std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame,
                                                 const cv::Mat& right_frame,
                                                 const std::vector<fsai::types::BoxBound>& box_bounds,
                                                 const cv::Ptr<cv::SIFT> sift_detector) {
    std::vector<ConeMatches> all_cone_matches;
    all_cone_matches.reserve(box_bounds.size());

    std::vector<PseudoFeature> left_features;
    left_features.reserve(box_bounds.size() * 8);
    const cv::Rect image_bounds(0, 0, left_frame.cols, left_frame.rows);

    for (int i = 0; i < static_cast<int>(box_bounds.size()); ++i) {
        const auto& box = box_bounds[i];
        const cv::Rect box_rect(static_cast<int>(box.x), static_cast<int>(box.y),
                                static_cast<int>(box.w), static_cast<int>(box.h));
        const cv::Rect clipped_rect = box_rect & image_bounds;
        if (clipped_rect.area() <= 0) {
            continue;
        }

        cv::Mat box_roi = left_frame(clipped_rect);

        std::vector<cv::KeyPoint> left_keypoints;
        cv::Mat left_descriptors;
        sift_detector->detectAndCompute(box_roi, cv::noArray(), left_keypoints, left_descriptors);
        if (left_descriptors.empty()) {
            continue;
        }

        for (int j = 0; j < static_cast<int>(left_keypoints.size()); ++j) {
            auto& kp = left_keypoints[j];
            kp.pt.x += static_cast<float>(clipped_rect.x);
            kp.pt.y += static_cast<float>(clipped_rect.y);

            PseudoFeature left_feature;
            left_feature.cone_index = i;
            left_feature.x = kp.pt.x;
            left_feature.y = kp.pt.y;
            left_feature.descriptor = left_descriptors.row(j).clone();
            left_features.push_back(left_feature);
        }
    }

    std::vector<cv::KeyPoint> right_keypoints;
    cv::Mat right_descriptors;
    sift_detector->detectAndCompute(right_frame, cv::noArray(), right_keypoints, right_descriptors);

    std::unordered_map<int, std::vector<PseudoFeature>> y_map;
    for (int i = 0; i < static_cast<int>(right_keypoints.size()); ++i) {
        if (right_descriptors.empty() || i >= right_descriptors.rows) {
            break;
        }

        PseudoFeature right_feature;
        right_feature.x = right_keypoints[i].pt.x;
        right_feature.y = right_keypoints[i].pt.y;
        right_feature.descriptor = right_descriptors.row(i);
        right_feature.cone_index = 0;

        const int y_key = static_cast<int>(right_feature.y);
        y_map[y_key].push_back(right_feature);
    }

    std::unordered_map<int, std::vector<Feature>> cone_map;

    for (const auto& left_feature : left_features) {
        const int y_key = static_cast<int>(left_feature.y);
        float min_score = std::numeric_limits<float>::max();
        std::optional<PseudoFeature> best_match;

        for (int y = y_key - kEpipolarMargin; y <= y_key + kEpipolarMargin; ++y) {
            auto it = y_map.find(y);
            if (it == y_map.end()) {
                continue;
            }

            const auto& possible_matches = it->second;
            for (const auto& possible_match : possible_matches) {
                float score = L2_score(left_feature.descriptor, possible_match.descriptor);
                if (!best_match || score < min_score) {
                    min_score = score;
                    best_match = possible_match;
                }
            }
        }

        if (!best_match) {
            continue;
        }

        Feature temp_feature;
        temp_feature.cone_index = left_feature.cone_index;
        temp_feature.x_1 = left_feature.x;
        temp_feature.y_1 = left_feature.y;
        temp_feature.x_2 = best_match->x;
        temp_feature.y_2 = best_match->y;

        cone_map[left_feature.cone_index].push_back(temp_feature);
    }

    for (const auto& [cone_index, matches] : cone_map) {
        ConeMatches temp_cone_match{};
        temp_cone_match.cone_index = cone_index;
        temp_cone_match.bound = box_bounds[cone_index];
        temp_cone_match.matches = matches;
        temp_cone_match.side = box_bounds[cone_index].side;
        all_cone_matches.push_back(temp_cone_match);
    }
    return all_cone_matches;
}
