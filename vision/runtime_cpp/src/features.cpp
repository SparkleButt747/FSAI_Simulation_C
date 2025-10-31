#include <iostream>
#include <features.hpp> 
#include <vector> 
// high level overview 
// we need function to take left and right frames and bounding box and return xy coordinates of matched features for each bounding box


cv::Mat extract_boximg(cv::Mat left_frame, BoxBound box_bound){
    // should be quite straight forward
    // we extract the bounding box image from the left frame
    // we return an opencv matrix whcih is a matrix of colour values 
}

cv::Mat project_right(cv::Mat left_boximg, cv::Mat right_frame){
    // we apply pnp here 
    // not sure how to implement this without having to do stereo triangulation...
    // we take the opencv matrix from extractboximg and apply PnP 
    // this projects the bounding box onto the right frame 
    // maybe it could work by just taking the top left point, then using same width and height 
    // maybe needs stereotriangulation???
}

std::vector<feature> bound_matching(cv::Mat left_boximg, cv::Mat right_boximg){
    // we apply feature matching between bounding box 
    // feature matching algorithm tbd 
    // this extracts xy coordinates relative to top left of bounding box
    // pass into fix_relativity to make it relative to left frame and right frame 
}

std::vector<feature> fix_relativity(std::vector<feature> unfixed_features){
    // we turn the features extracted from bounding boxes from relative to bounding box to relative to frame 
    // should be quite straight forward  
}

std::vector<feature> match_features(cv::Mat left_frame, cv::Mat right_frame,std::vector<BoxBound> box_bounds){
    // loop over each bounding box
    // extract the bounding box image in left frame 
    // apply PnP to estimate bounding box in right frame 
    // take the two bounding box images and apply feature matching algorithm (TBD)
    // append features to vector 
    // return vector 
}