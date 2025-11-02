#include <iostream>
#include "features.hpp"
#include "vector"
#include <detect.hpp>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// high level overview 
// we need function to take left and right frames and bounding box and return xy coordinates of matched features for each bounding box

cv::Mat extract_boundimg(cv::Mat left_frame, BoxBound box_bound){
    // extracts the bounding box image for a single bounding box object 
    cv::Rect roi(box_bound.x, box_bound.y, box_bound.w, box_bound.h);   
    cv::Mat box_boundimg = left_frame(roi);                               
    return box_boundimg;                                               

}

std::vector<pseudofeature> extract_features(cv::Mat frame, cv::Ptr<cv::ORB> orb){
    // extract all features from a frame. If using for left frame, pass in results from extract_boundimg 
    // each pseudofeature has xy coordinates in coordinate system of right frame, and a descriptor matrix used for ORB or SIFT 

    // declare keypoints and descriptors matrix
    std::vector<KeyPoint> keypoints; 
    cv::Mat descriptors; 

    // detect and compute
    orb->detect(frame, keypoints); 
    orb->compute(frame, keypoints, descriptors);

    std::vector<pseudofeature> results; 

    for (int i = 0; i < keypoints.size(); i++){
        KeyPoint keypoint_i = keypoints[i];
        cv::Mat descriptor_i = descriptors.row(i);

        //keypoint has attribute pt, which is (x,y)
        int x = keypoint_i.pt.x;
        int y = keypoint_i.pt.y;

        // initialise new pseudofeature and append
        pseudofeature pseudofeature_i{x,y,descriptor_i};
        results.push_back(pseudofeature_i);
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

    //define matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING, true); 

    //try to find matching features 
    for (int i = 0; i < left_features.size(); i++){
        pseudofeature left_feature_i = left_features[i];
        int y = left_feature_i.y;
        int x = left_feature_i.x;

        //temp variable to hold possible matches 
        std::vector<pseudofeature> possible_matches; 

        // look for possible matches within tolerance
        for (int j = y-epipolar_tolerance; j < y+epipolar_tolerance; j++){
            std::vector<pseudofeature> fetched_features = y_mapping[j];
            possible_matches.insert(possible_matches.end(), fetched_features.begin(), fetched_features.end());
        }

        int best_x, best_y; 
        int min = -1; 
        
        // for each possible match, compute hamming distance and find lowest
        for (int j = 0; j < possible_matches.size(); j++){
            pseudofeature right_feature_j = possible_matches[j];
            int right_x = right_feature_j.x;
            int right_y = right_feature_j.y;
            cv::Mat left_descriptor, right_descriptor = left_feature_i.descriptors, right_feature_j.descriptors;

            int hamming_distance = cv::norm(left_descriptor, right_descriptor, cv::NORM_HAMMING);

            if (min == -1){
                min = hamming_distance; 
                best_x, best_y = right_x, right_y; 
            }
            else if (hamming_distance < min){
                min = hamming_distance;
                best_x, best_y = right_x, right_y; 
            }
            else{
                // do nothing 
            }
        }

        feature result_feature{x,y,best_x,best_y};
        results.push_back(result_feature);
    }
    return results
}

std::vector<feature> extract_coordinates(std::vector<std::tuple<feature>> pairs){
    // we format the pseudofeatures into actual features which look like (x1,y1,x2,y2)
    // x1y1 are coordinates in left frame and x2y2 are coordinates in right frame
}


std::vector<feature> match_features(cv::Mat left_frame, cv::Mat right_frame,std::vector<BoxBound> box_bounds){
    // loop over each bounding box
    // extract the bounding box image in left frame 
    // apply PnP to estimate bounding box in right frame 
    // take the two bounding box images and apply feature matching algorithm (TBD)
    // append features to vector 
    // return vector 

    cv::Ptr<cv::ORB> orb = cv::ORB::create();                                                  // create orb object to pass into extraction functions
}

int test_feature_matching(cv::Mat left_frame, cv::Mat right_frame){
    // test the feature matching based on left frame and right frame 

}