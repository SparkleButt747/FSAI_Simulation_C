#pragma once
#include "common/include/common/types.h"
#include <vector>
#include <opencv2/opencv.hpp> 

struct BoxBound{
    int x,y;
    float w,h;
    float conf;
    fsai::types::ConeSide side;
};

std::vector<BoxBound>detect_cones(cv::Mat image);