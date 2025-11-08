#pragma once
#include <vector> 
#include <opencv4/opencv2/opencv.hpp>
#include "detect.hpp"
struct feature
{
    /* data */
    double x_1,y_1;
    double x_2,y_2;
};

struct pseudofeature{
    double x,y; 
    cv::Mat descriptors; 
};

cv::Mat extract_boundimg(cv::Mat left_frame, fsai::vision::BoxBound box_bound);
std::vector<pseudofeature> extract_feature(fsai::vision::BoxBound box_bound);
std::vector<pseudofeature> extract_right_features(cv::Mat right_frame);
std::vector<std::tuple<pseudofeature>> pair_features(std::vector<feature> left_features, std::vector<feature> right_feature);
std::vector<feature> extract_coordinates(std::vector<std::tuple<feature>> pairs);
std::vector<feature> match_features(cv::Mat left_frame, cv::Mat right_frame,std::vector<fsai::vision::BoxBound> box_bounds);

