#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "vision/detect.hpp"

using fsai::vision::BoxBound;
struct feature
{
    /* data */
    double x_1,y_1;
    double x_2,y_2;
};

std::vector<feature> match_features(cv::Mat left_frame, cv::Mat right_frame,std::vector<BoxBound> box_bounds);
