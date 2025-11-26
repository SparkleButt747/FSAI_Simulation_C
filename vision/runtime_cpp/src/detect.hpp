#pragma once
#include "common/include/common/types.h"
#include "cone_tracker.hpp"
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp> 

namespace fsai{
namespace vision{
class ConeDetector{
    public:

    ConeDetector(const std::string& path_to_model);

    ~ConeDetector();

    ConeDetector(const ConeDetector&) = delete;
    ConeDetector& operator = (const ConeDetector&) = delete;
    ConeDetector(ConeDetector&&) = delete;
    ConeDetector& operator = (ConeDetector&&) = delete;

    

    std::vector<types::BoxBound>detectCones(const cv::Mat& left_frame);

    private:
    ConeTracker tracker_;

    //members
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_; //this is the "model"
};

}
}
