#include <iostream>
#include "features.hpp"
#include "vector"
#include <detect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// high level overview 
// we need function to take left and right frames and bounding box and return xy coordinates of matched features for each bounding box

cv::Mat extract_boundimg(cv::Mat left_frame, BoxBound box_bound){
    // extracts the bounding box image for a single bounding box object 
    cv::Rect roi(box_bound.x, box_bound.y, box_bound.w, box_bound.h);   // define region of interest for extraction 
    cv::Mat box_boundimg = left_frame(roi);                               // extracts the region we want as a reference (we never modify it anyways, no need to clone)
    return box_boundimg;                                                // returns a reference to the part of the frame we want 

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
        keypoint_i = keypoints[i]
        descriptor_i = descriptors[i]

        //keypoint has attribute pt, which is (x,y)
        x = keypoint_i.pt[0] 
        y = keypoint_i.pt[1]

        // initialise new pseudofeature and append
        pseudofeature pseudofeature_i{x,y,descriptor_i}
        results.push_back(pseudofeature_i)
    }

    return results // should be a vector of pseudofeatures, e.g. [(x,y,descriptor), (x,y,descriptor), ...]
}

std::vector<std::tuple<pseudofeature>> pair_features(std::vector<feature> left_features, std::vector<feature> right_feature){
    // use epipolar constraint to search along epipolar line for each left feature to find matching right feature 
    // return an "array" of (left_feature, right_feature) to pass for coordinate calculation 
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