#include "features.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>  // For SIFT and other feature detectors
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <iostream>

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
    
    std::vector<Feature> matched_features; 
    std::vector<PseudoFeature> left_features;
    std::vector<PseudoFeature> right_features;

    for (size_t i = 0; i < box_bounds.size(); ++i) {
    	const auto& box = box_bounds[i];
    	
    	cv::Rect box_roi(box.x, box.y, box.w, box.h);
    	cv::Ptr<cv::SIFT> sift_detector = cv::SIFT::create();  
    	
    	if (!is_safe_roi(left_frame, box_roi)){
    	    continue;
    	}
    	else{
    	    // crop the roi from left_frame
    	    cv::Mat left_roi;
    	    left_roi = left_frame(box_roi);
    	    
    	    // initialise holders for SIFT outputs 
    	    std::vector<cv::KeyPoint> left_keypoints;
    	    cv::Mat left_descriptors; 
    	    
    	    // extract features from roi 
    	    sift_detector->detectAndCompute(left_roi, cv::noArray(), left_keypoints, left_descriptors);
    	    
    	    for (size_t j = 0; j < left_keypoints.size(); ++j){
    	        // fetch and fix relativity, make it relative to entire left frame 
    	        cv::KeyPoint left_keypoint = left_keypoints[j];
    	        left_keypoint.pt.x += box.x;
    	        left_keypoint.pt.y += box.y;
    	        
    	        // save feature 
    	        PseudoFeature left_feature;
    	        left_feature.cone_index = i; 
    	        left_feature.x = left_keypoint.pt.x;
    	        left_feature.y = left_keypoint.pt.y;
    	        left_feature.descriptor = left_descriptors[j];
    	        left_features.push_back(left_feature);
    	    }   
    	}
    }
    
    //feature extraction from right frame
    std::vector<cv::KeyPoint> right_keypoints;
    cv::Mat right_descriptors;
    sift_detector->detectAndCompute(right_frame, cv::noArray(), right_keypoints, right_descriptors);
    std::unordered_map<int, std::vector<PseudoFeature>> y_map;
    
    for (size_t i = 0; i < right_keypoints.size(); ++i){
        cv::KeyPoint right_keypoint = right_keypoints[i];
        
        PseudoFeature right_feature; 
        right_feature.cone_index = 0;  // we do not know what cone it belongs to or whether it is a cone at all 
        right_feature.x = right_keypoint.pt.x;
        right_feature.y = right_keypoint.pt.y;
        right_feature.descriptor = right_descriptors[i];
        right_features.push_back(right_feature);
        y_map[right_keypoint.pt.y].push_back(right_feature);
    }
    
    for (size_t i = 0; i < left_features.size(); ++i){
        PseudoFeature left_feature = left_features[i];
        int y = left_feature.y;
        bool key_exists = y_map.find(y) != y_map.end();
        if (key_exists){
            std::vector<PseudoFeature> possible_matches = y_map[y];
            PseudoFeature best_match;
            double min_score = 999; 
            
            // go through each PseudoFeature in possible_matches 
            // score using L2 
            // if score < min_score, update min score and best_match 
            // after iteration, create new feature 
            // push feature to matched_features
            
        }
    }
    
    // create new ConeMatches for each cone 
    // for each matched feature, 
    // push the feature into ConeMatches[cone_index]
    // give box_bound[i] to ConeMatches
    // give cone_index to ConeMatches
    
    
    
    

    return all_cone_matches;
}
