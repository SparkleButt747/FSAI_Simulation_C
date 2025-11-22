#include "features.hpp"
#include "detect.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>  // For SIFT and other feature detectors
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <limits> 
#include <iostream>

// TUNING: Epipolar search margin (pixels up/down to look in right image)
static const int kEpipolarMargin = 2;

bool is_safe_roi(const cv::Mat& frame, cv::Rect roi) {
    return (roi.x >= 0 && roi.y >= 0 && 
            roi.x + roi.width <= frame.cols && 
            roi.y + roi.height <= frame.rows &&
            roi.width > 0 && roi.height > 0);
}

float L2_score(cv::Mat left_descriptor, cv::Mat right_descriptor){
    float dist = cv::norm(left_descriptor, right_descriptor, cv::NORM_L2); 
    return dist; 
}

std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame, 
                                                 const cv::Mat& right_frame, 
                                                 const std::vector<fsai::vision::BoxBound>& box_bounds) {
    std::vector<ConeMatches> all_cone_matches;
    all_cone_matches.reserve(box_bounds.size());
    
    cv::Ptr<cv::SIFT> sift_detector = cv::SIFT::create(); 
    
    std::vector<PseudoFeature> left_features; 
    std::vector<PseudoFeature> right_features; 
    
    for (int i = 0; i < box_bounds.size(); ++i){
        // get information about the box 
        fsai::vision::BoxBound box_i = box_bounds[i]; 
        int box_index = i; 
        
        // get roi 
        cv::Rect box_rect(box_i.x, box_i.y, box_i.w, box_i.h); 
        cv::Mat box_roi; 
        box_roi = left_frame(box_rect); 
        if (!is_safe_roi(left_frame, box_rect)){continue;}
        
        std::vector<cv::KeyPoint> left_keypoints; 
        cv::Mat left_descriptors; 
        
        sift_detector->detectAndCompute(box_roi, cv::noArray(), left_keypoints, left_descriptors); 
        
        for (int j = 0; j < left_keypoints.size(); ++j){
            //fix relativity - make it relative to entire left frame 
            left_keypoints[j].pt.x += box_i.x; 
            left_keypoints[j].pt.y += box_i.y;  
            
            PseudoFeature left_feature;
            left_feature.cone_index = box_index;
            left_feature.x = left_keypoints[j].pt.x; 
            left_feature.y = left_keypoints[j].pt.y; 
            left_feature.descriptor = left_descriptors.row(j).clone(); 
            left_features.push_back(left_feature); 
        }
        
    }
    
    std::vector<cv::KeyPoint> right_keypoints; 
    cv::Mat right_descriptors; 
    sift_detector->detectAndCompute(right_frame, cv::noArray(), right_keypoints, right_descriptors); 
    
    std::unordered_map<int, std::vector<PseudoFeature>> y_map; 
    
    // format features extracted from right frame and build hash map 
    for (int i = 0; i < right_keypoints.size(); ++i){
    
        PseudoFeature right_feature; 
        right_feature.x = right_keypoints[i].pt.x;
        right_feature.y = right_keypoints[i].pt.y; 
        right_feature.descriptor = right_descriptors.row(i).clone(); 
        right_feature.cone_index = 0; 
        right_features.push_back(right_feature); 
        
        int y_key = cvRound(right_feature.y);
        y_map[y_key].push_back(right_feature);  
    }
    
    std::unordered_map<int, std::vector<Feature>> cone_map; 
    
    // for each left_feature, search for best match for y +- tolerance in hash map
    for (int i = 0; i < left_features.size(); ++i){
        PseudoFeature left_feature_i = left_features[i]; 
        int y = left_feature_i.y; 
        cv::Mat left_descriptor = left_feature_i.descriptor; 
        float minScore = std::numeric_limits<float>::max();   // init value 
        PseudoFeature best_match; 
        
        for (int j = y - kEpipolarMargin; j < y + kEpipolarMargin; ++j){
            bool key_exists = y_map.find(j) != y_map.end(); 
            if (!key_exists){continue;}
            
            const auto& possible_matches = y_map[j];
            
            for (int k = 0; k < possible_matches.size(); ++k){
            	PseudoFeature possible_match = possible_matches[k]; 
            	float score = L2_score(left_descriptor, possible_match.descriptor); 
            	if (score < minScore){
            	    minScore = score; 
            	    best_match = possible_match; 
            	}
            }
        }
// IMPORTANT : THE ABOVE CODE IS TESTED IN SEPARATE ENVIRONMENT AND SHOULD WORK
// only edited the part for iterating over box bounds, otherwise it's the same     
// IMPORTANT : THE FOLLOWING CODE IS UNTESTED IN PRACTISE. 
        
        int x1 = left_feature_i.x; 
        int y1 = left_feature_i.y; 
        int x2 = best_match.x; 
        int y2 = best_match.y; 
        //Case no matches were found in epipolar range.
        if (minScore == std::numeric_limits<float>::max()) continue;

        
        Feature tempFeature;
        tempFeature.cone_index = left_feature_i.cone_index; 
        tempFeature.x_1 = x1; 
        tempFeature.x_2 = x2; 
        tempFeature.y_1 = y1; 
        tempFeature.y_2 = y2; 
        
        cone_map[left_feature_i.cone_index].push_back(tempFeature); 
    }
    
    for (const auto& pair: cone_map){
        ConeMatches tempConeMatch;
        
        int cone_index = pair.first;  
    
        tempConeMatch.cone_index = cone_index; 
        tempConeMatch.bound = box_bounds[cone_index];
        tempConeMatch.matches = pair.second; 
        
        all_cone_matches.push_back(tempConeMatch);
    }
    return all_cone_matches;
}

std::vector<PseudoFeature> extract_features(const cv::Mat& frame, cv::Ptr<cv::SIFT> sift) {
    
}
