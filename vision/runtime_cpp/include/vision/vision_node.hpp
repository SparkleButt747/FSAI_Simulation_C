#pragma once
#ifndef FSAI_VISION_VISION_NODE_HPP
#define FSAI_VISION_VISION_NODE_HPP

#include "common/include/common/types.h"
#include "detect.hpp"
#include "features.hpp"
#include "shared_ring_buffer.hpp"


#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>


namespace fsai{
namespace vision{
struct RenderableFrame {
    cv::Mat image;
    std::vector<BoxBound> boxes;
    uint64_t timestamp_ns = 0;
    bool valid = false;
};
struct ConeCluster{
    int coneId;
    std::vector<Eigen::Vector3d> points;
};
// struct Point3D{
//     double X,Y,Z;
// };
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
   using PoseProvider = std::function<std::pair<Eigen::Vector2d, double>()>;
   void setPoseProvider(PoseProvider provider){pose_provider_ = provider;}
   std::optional<fsai::types::Detections> makeDetections();
   std::optional<fsai::types::Detections> getLatestDetections();
   RenderableFrame getRenderableFrame();
   

   private:

    void runProcessingLoop();
    // private helpers
    inline bool triangulatePoint(const Feature& feat,Eigen::Vector3d& result);
    cv::Mat frameToMat(const fsai::types::Frame& frame);

    // private members
    PoseProvider pose_provider_;
    bool intrinsics_set_ = false;
    FsaiCameraIntrinsics cameraParams_;
    const double BASE_LINE_ = 0.2;
    std::unique_ptr<fsai::vision::SimCamera> camera_;
    std::unique_ptr<fsai::vision::ConeDetector> detector_;

    std::thread processing_thread_;
    std::atomic<bool> running_{false};

    std::mutex detection_mutex_;
    std::optional<fsai::types::Detections> latest_detections_;
    std::mutex render_mutex_;
    RenderableFrame latest_renderable_frame_;

    //Add ring buffer type 
    using DetectionsRingBuffer = fsai::vision::GenericRingBuffer<fsai::types::Detections>;
    std::shared_ptr<DetectionsRingBuffer> detection_buffer_;


};
}
}
#endif