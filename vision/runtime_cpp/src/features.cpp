#include "features.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>

// TUNING: Patch size for matching individual points. 
// 5x5 or 7x7 is good for small 11x13 cones.
static const int kPatchSize = 5; 
static const int kHalfPatch = kPatchSize / 2;

// TUNING: How many pixels to skip when sampling the grid (1 = every pixel, 2 = every other)
static const int kGridStride = 3; 

// TUNING: Epipolar search margin (pixels up/down to look in right image)
static const int kEpipolarMargin = 2;

bool is_safe_roi(const cv::Mat& frame, cv::Rect roi) {
    return (roi.x >= 0 && roi.y >= 0 && 
            roi.x + roi.width <= frame.cols && 
            roi.y + roi.height <= frame.rows &&
            roi.width > 0 && roi.height > 0);
}

std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame, 
                                                 const cv::Mat& right_frame, 
                                                 const std::vector<fsai::vision::BoxBound>& box_bounds) {
    std::vector<ConeMatches> all_cone_matches;
    all_cone_matches.reserve(box_bounds.size());

    // Pre-allocate standardized patch matrices to avoid re-allocation in loop
    cv::Mat left_patch, right_strip, res_map;

    for (size_t i = 0; i < box_bounds.size(); ++i) {
        const auto& box = box_bounds[i];
        std::vector<Feature> cone_features;

        // 1. Define safe boundaries for sampling points inside the box
        // We must ensure a kPatchSize window can fit around the point.
        int start_x = box.x + kHalfPatch;
        int end_x = box.x + box.w - kHalfPatch;
        int start_y = box.y + kHalfPatch;
        int end_y = box.y + box.h - kHalfPatch;

        if (start_x >= end_x || start_y >= end_y) continue; // Box too small for patches

        // 2. Grid Sample inside the Left Cone Box
        for (int ly = start_y; ly < end_y; ly += kGridStride) {
            for (int lx = start_x; lx < end_x; lx += kGridStride) {
                
                // Extract small patch around grid point (lx, ly)
                cv::Rect patch_roi(lx - kHalfPatch, ly - kHalfPatch, kPatchSize, kPatchSize);
                if (!is_safe_roi(left_frame, patch_roi)) continue;
                left_patch = left_frame(patch_roi);

                // Define search strip in Right image
                // Constrain Y to epipolar line +/- margin
                int strip_y = std::max(0, ly - kHalfPatch - kEpipolarMargin);
                int strip_h = kPatchSize + (kEpipolarMargin * 2);
                // Constrain X: matched point must be to the left of lx (or same pos for infinity)
                int strip_width = std::min(right_frame.cols, lx + kHalfPatch + 5); // +5 for slight tolerance
                
                cv::Rect strip_roi(0, strip_y, strip_width, strip_h);
                if (!is_safe_roi(right_frame, strip_roi)) continue;
                right_strip = right_frame(strip_roi);

                // Match the small patch
                // TM_SQDIFF is faster than NORMED for tiny patches, but less robust to lighting changes.
                // Sim lighting is usually even, so SQDIFF might be okay. Using NORMED for safety.
                cv::matchTemplate(right_strip, left_patch, res_map, cv::TM_SQDIFF_NORMED);
                
                double min_val; cv::Point min_loc;
                cv::minMaxLoc(res_map, &min_val, nullptr, &min_loc, nullptr);

                // Threshold: Only accept good patch matches (e.g., < 0.1 difference)
                if (min_val < 0.1) {
                    Feature feat;
                    feat.x_1 = static_cast<double>(lx);
                    feat.y_1 = static_cast<double>(ly);
                    // Convert strip-local coordinate back to global right-image coordinate
                    feat.x_2 = static_cast<double>(strip_roi.x + min_loc.x + kHalfPatch);
                    feat.y_2 = static_cast<double>(strip_roi.y + min_loc.y + kHalfPatch);

                    // Final check: ensure positive disparity (closer objects shift left)
                    if (feat.x_2 <= feat.x_1 + 1.0) { // +1.0 tolerance for noise
                         cone_features.push_back(feat);
                    }
                }
            }
        }

        if (!cone_features.empty()) {
            all_cone_matches.push_back({(int)i, box, cone_features,box.side});
        }
    }

    return all_cone_matches;
}