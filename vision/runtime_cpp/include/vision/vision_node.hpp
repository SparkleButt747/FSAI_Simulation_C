#pragma once
#ifndef FSAI_VISION_VISION_NODE_HPP
#define FSAI_VISION_VISION_NODE_HPP

#include "common/include/common/types.h"


#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>



namespace fsai{
namespace vision{

class SimCamera;
class ConeDetector;
class VisionNode{
    public:
    /**
     * @brief Constructs the VisionNode.
     * This will initialise the SimCamera/ZED camera for the real thing
     * Intialises the .onnx yolo model
     * Will throw if the setup fails
     */
    VisionNode();

    /**
     * @brief Destructor
     * Stops the processing thread.
     */
    ~VisionNode();

    VisionNode(const VisionNode&) = delete;
    VisionNode& operator = (const VisionNode&) = delete;
    VisionNode(VisionNode&&) = delete;
    VisionNode& operator = (VisionNode&&) = delete;

    /**
     * @brief Starts the internal processing thread
     */
    void start();

    /**
     * @brief stops the internal processing thread
    */
   void stop();

   std::optional<fsai::types::Detections> makeDetections();

   private:

    void runProcessingLoop();

    cv::Mat frameToMat(const fsai::types::Frame& frame);

    std::unique_ptr<fsai::vision::SimCamera> camera_;
    std::unique_ptr<fsai::vision::ConeDetector> detector_;

    std::thread processing_thread_;
    std::atomic<bool> running_{false};

    std::mutex detection_mutex_;
    std::optional<fsai::types::Detections> latest_detections_;

};
}
}
#endif