#include "features.hpp"
#include "common/include/common/types.h"
#include "detect.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>  // For SIFT and other feature detectors
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <limits>
#include <iostream>
#include <optional>
#include "common/types.h"

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

int hamming_dist(cv::Mat left_descriptor, cv::Mat right_descriptor){
    int dist = cv::norm(left_descriptor,  right_descriptor, cv::NORM_HAMMING); 
    return dist; 
}

std::vector<ConeMatches> match_features_per_cone(const cv::Mat& left_frame, 
                                                 const cv::Mat& right_frame, 
                                                 const std::vector<fsai::types::BoxBound>& box_bounds,
                                                 const cv::Ptr<cv::ORB> detector) {
    std::vector<ConeMatches> all_cone_matches;
    all_cone_matches.reserve(box_bounds.size());
    
    std::vector<PseudoFeature> left_features; 
    
    left_features.clear();
   
    
    std::vector<cv::KeyPoint> left_keypoints; 
    cv::Mat left_descriptors; 
    detector->detectAndCompute(left_frame, cv::noArray(), left_keypoints, left_descriptors); 
    
    for (int i = 0; i < left_keypoints.size(); i++){
    	cv::KeyPoint left_keypoint_i = left_keypoints[i]; 
    	int cone_index = 0; 
    	
    	bool is_cone_kp = false; 
    	
    	for (int j = 0; j < box_bounds.size(); j++){
    	    fsai::types::BoxBound box_j = box_bounds[j]; 
            int box_index = j; 
            cv::Rect box_rect(box_j.x, box_j.y, box_j.w, box_j.h); 
            if (box_rect.contains(left_keypoint_i.pt)){
            	is_cone_kp = true; 
            	cone_index = j; 
            	break;
            }
    	}
    	
    	if (is_cone_kp){
    	    PseudoFeature left_feature; 
    	    left_feature.x = left_keypoint_i.pt.x;
    	    left_feature.y = left_keypoint_i.pt.y;
    	    left_feature.descriptor = left_descriptors.row(i).clone(); 
    	    left_feature.cone_index = cone_index;
    	    left_features.push_back(left_feature);
    	}
    }
    
    std::vector<cv::KeyPoint> right_keypoints;
    cv::Mat right_descriptors;
    detector->detectAndCompute(right_frame, cv::noArray(), right_keypoints, right_descriptors);

    std::unordered_map<int, std::vector<PseudoFeature>> y_map;

    // format features extracted from right frame and build hash map
    for (int i = 0; i < right_keypoints.size(); ++i){

        PseudoFeature right_feature;
        right_feature.x = right_keypoints[i].pt.x;
        right_feature.y = right_keypoints[i].pt.y; 
        right_feature.descriptor = right_descriptors.row(i).clone(); 
        right_feature.cone_index = 0; 
        
        y_map[right_feature.y].push_back(right_feature);  
    }
    
    std::unordered_map<int, std::vector<Feature>> cone_map;
    
    // for each left_feature, search for best match for y +- tolerance in hash map
    for (int i = 0; i < left_features.size(); ++i){
        const PseudoFeature& left_feature_i = left_features[i]; 
        int y = left_feature_i.y; 
        const auto& left_descriptor = left_feature_i.descriptor;   // try changing cv::Mat to const auto &
        float minScore = std::numeric_limits<float>::max();   // init value 
        PseudoFeature best_match; 
        
        for (int j = y - kEpipolarMargin; j < y + kEpipolarMargin; ++j){
            bool key_exists = y_map.find(j) != y_map.end();
            if (!key_exists){continue;}
            
            const auto& possible_matches = y_map[j]; 
            
            for (int k = 0; k < possible_matches.size(); ++k){
            	PseudoFeature possible_match = possible_matches[k];
            	int score = hamming_dist(left_descriptor, possible_match.descriptor);
            	if (score < minScore){
            	    minScore = score;
            	    best_match = possible_match;
            	}
            }
        }

        int x1 = left_feature_i.x;
        int y1 = left_feature_i.y;
        int x2 = best_match.x;
        int y2 = best_match.y;

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
        tempConeMatch.side = box_bounds[cone_index].side;
        
        all_cone_matches.push_back(tempConeMatch);
    }
    return all_cone_matches;
}
