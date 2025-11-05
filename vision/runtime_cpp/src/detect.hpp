#pragma once
#include "common/include/common/types.h"
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp> 

namespace fsai{
namespace vision{
struct BoxBound{
    int x,y;
    float w,h;
    float conf;
    fsai::types::ConeSide side;
};
class ConeDetector{
    public:

    ConeDetector(const std::string& path_to_model);

    ~ConeDetector();

    ConeDetector(const ConeDetector&) = delete;
    ConeDetector& operator = (const ConeDetector&) = delete;
    ConeDetector(ConeDetector&&) = delete;
    ConeDetector& operator = (ConeDetector&&) = delete;

    

    std::vector<fsai::types::ConeDet> detectCones(const cv::Mat& left_frame);

    private:

    fsai::types::ConeDet processDetection(const BoxBound& box_bound);
    //members
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_; //this is the "model"
};

}
}
