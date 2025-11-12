#include "features.hpp"
<<<<<<< HEAD
#include "vector"
#include "detect.hpp"
#include <unordered_map>
#include <opencv2/opencv.hpp>

=======
#include <opencv2/imgproc.hpp>
#include <iostream>
>>>>>>> dev-connect-vis-ctrl

// TUNING: Patch size for matching individual points. 
// 5x5 or 7x7 is good for small 11x13 cones.
static const int kPatchSize = 5; 
static const int kHalfPatch = kPatchSize / 2;

<<<<<<< HEAD
namespace fsai {
    namespace vision{
        cv::Mat extract_boundimg(cv::Mat left_frame, BoxBound box_bound){
            // extracts the bounding box image for a single bounding box object
            cv::Rect roi(box_bound.x, box_bound.y, box_bound.w, box_bound.h);   // define region of interest for extraction

            //Clip the roi to be in frame dimensions
            roi &= cv::Rect(0,0, left_frame.cols, left_frame.rows);

            if (roi.width <= 0 || roi.height <= 0) {
                std::cerr << "Warning: Out of bands ROI requested." << std::endl;
                return cv::Mat();
            }
            cv::Mat box_boundimg = left_frame(roi);                               // extracts the region we want as a reference (we never modify it anyways, no need to clone)
            return box_boundimg;                                                // returns a reference to the part of the frame we want

        }

        std::vector<pseudofeature> extract_features(cv::Mat frame, cv::Ptr<cv::ORB> orb){
            // extract all features from a frame. If using for left frame, pass in results from extract_boundimg
            // each pseudofeature has xy coordinates in coordinate system of right frame, and a descriptor matrix used for ORB or SIFT

            //Empty frame check
            if (frame.empty()) {
                std::cerr << "Warning: Empty frame received" << std::endl;
                return {};
            }

            // declare keypoints and descriptors matrix
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;

            // detect and compute
            orb->detect(frame, keypoints);
            orb->compute(frame, keypoints, descriptors);

            std::vector<pseudofeature> results;

            for (int i = 0; i < keypoints.size(); i++){
                cv::KeyPoint keypoint_i = keypoints[i];
                cv::Mat descriptor_i = descriptors.row(i);

                //keypoint has attribute pt, which is (x,y)
                int x = keypoint_i.pt.x;
                int y = keypoint_i.pt.y;

                // initialise new pseudofeature and append
                pseudofeature pseudofeature_i{(double)x,(double)y,descriptor_i};
                results.push_back(pseudofeature_i);
=======
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
>>>>>>> dev-connect-vis-ctrl
            }

            return results; // should be a vector of pseudofeatures, e.g. [(x,y,descriptor), (x,y,descriptor), ...]
        }

        std::vector<feature> pair_features(std::vector<pseudofeature> left_features, std::vector<pseudofeature> right_features){
            // use epipolar constraint to search along epipolar line for each left feature to find matching right feature
            // return an "array" of (left_feature, right_feature) to pass for coordinate calculation


            //define output
            std::vector<feature> results;

            //create dictionary for y-hash for quick lookup
            std::unordered_map<int, std::vector<pseudofeature>> y_mappings = {};

            //add right_features to hash table
            for (int i = 0; i < right_features.size(); i++){
                pseudofeature right_feature_i = right_features[i];
                int y = right_feature_i.y;
                y_mappings[y].push_back(right_feature_i);
            }

            //define how much to search up and down, e.g. +-1 pixel if it is 1
            int epipolar_tolerance = 1;

            //try to find matching features
            for (int i = 0; i < left_features.size(); i++){
                pseudofeature left_feature_i = left_features[i];
                int y = left_feature_i.y;
                int x = left_feature_i.x;

                //temp variable to hold possible matches
                std::vector<pseudofeature> possible_matches;

                // look for possible matches within tolerance
                for (int j = y-epipolar_tolerance; j < y+epipolar_tolerance; j++){
                    std::vector<pseudofeature> fetched_features = y_mappings[j];
                    possible_matches.insert(possible_matches.end(), fetched_features.begin(), fetched_features.end());
                }

                int best_x, best_y;
                int min = -1;

                // for each possible match, compute hamming distance and find lowest
                for (int j = 0; j < possible_matches.size(); j++){
                    pseudofeature right_feature_j = possible_matches[j];
                    int right_x = right_feature_j.x;
                    int right_y = right_feature_j.y;
                    cv::Mat left_descriptor= left_feature_i.descriptors, right_descriptor = right_feature_j.descriptors;

                    int hamming_distance = cv::norm(left_descriptor, right_descriptor, cv::NORM_HAMMING);

                    if (min == -1){
                        min = hamming_distance;
                        best_x = right_x, best_y = right_y;
                    }
                    else if (hamming_distance < min){
                        min = hamming_distance;
                        best_x = right_x, best_y = right_y;
                    }
                    else{
                        // do nothing
                    }
                }

                feature result_feature{(double)x,(double)y,(double)best_x,(double)best_y};
                results.push_back(result_feature);
            }
            return results;
        }


        std::vector<feature> match_features(cv::Mat left_frame, cv::Mat right_frame,std::vector<BoxBound> box_bounds){

            //create orb object to feed into extraction functions
            cv::Ptr<cv::ORB> orb = cv::ORB::create();

            //create variable to hold left_features
            std::vector<pseudofeature> left_features;
            cv::Mat extracted_image;
            std::vector<pseudofeature> extracted_features;
            // go over each bounding box
            for (int i = 0; i < box_bounds.size(); i++){
                fsai::vision::BoxBound box_i = box_bounds[i];
                int x = box_i.x;
                int y = box_i.y;

                //extract bounding box image and its features
                extracted_image = extract_boundimg(left_frame, box_i);
                extracted_features = extract_features(extracted_image, orb);

                //make it relative to entire left frame instead of bounding box
                for (int j = 0; j < extracted_features.size();j++){
                    pseudofeature feature_i = extracted_features[i];
                    feature_i.x = feature_i.x + x;
                    feature_i.y = feature_i.y + y;
                    left_features.push_back(feature_i);
                }
            }

            //create variable to hold right_features
            std::vector<pseudofeature> right_features;
            right_features = extract_features(right_frame, orb);

            std::vector<feature> results;
            results = pair_features(left_features, right_features);

            return results;
        }

<<<<<<< HEAD
    }
}


int main(){
    // test the feature matching based on left frame and right frame
    std::cout << "Build Success!!!!";
    return 0;
}
=======
        if (!cone_features.empty()) {
            all_cone_matches.push_back({(int)i, box, cone_features});
        }
    }

    return all_cone_matches;
}
>>>>>>> dev-connect-vis-ctrl
