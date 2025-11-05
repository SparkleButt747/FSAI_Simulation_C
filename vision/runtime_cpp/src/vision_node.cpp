/**
 * @brief This file handles the joining of the whole vision pipeline from image to ConDet
 * TO-DO:
 * 
 */
#include "vision/vision_node.hpp"
#include "common/include/common/types.h"
#include "detect.hpp"
#include "sim_camera.hpp"
#include <iostream>
#include <chrono>

const char* PATH_TO_MODEL = "../../models/cone_model.onnx";
constexpr std::chrono::milliseconds kIdleSleep(5);
namespace fsai{
namespace vision{

VisionNode::VisionNode(){

    std::cout << "VisionNode: Initialisation ..." << std::endl;
    camera_ = std::make_unique<fsai::vision::SimCamera>();
    detector_ = std::make_unique<fsai::vision::ConeDetector>(PATH_TO_MODEL);
    std::cout << "VisionNode: Initialisation complete" << std::endl;
}

VisionNode::~VisionNode(){
    std::cout << "VisionNode: Shutting down ..." << std::endl;
    stop();
}

void VisionNode::start(){
    if(running_){
        return;
    }
    running_ = true;

    processing_thread_ = std::thread(&VisionNode::runProcessingLoop, this);
    std::cout << "VisionNode: Processing thread started" << std::endl;
}

void VisionNode::stop(){
    running_ = false;
    if(processing_thread_.joinable()){
        processing_thread_.join();
        std::cout << "VisionNode: Processing thread joined" << std::endl;
    }
}

std::optional<fsai::types::Detections> VisionNode::makeDetections(){
    std::lock_guard<std::mutex> lock(detection_mutex_); 
    return latest_detections_;
}

cv::Mat VisionNode::frameToMat(const fsai::types::Frame& frame){
    if(!frame.data||frame.w <=0 || frame.h <=0){
        return cv::Mat();
    }

    switch(frame.fmt){
        case FSAI_PIXEL_RGB888:{
            cv::Mat rgb_wrapper(frame.h,frame.w, CV_8UC3, frame.data,
                                static_cast<size_t>(frame.stride));
            cv::Mat bgr_mat;
            cv::cvtColor(rgb_wrapper,bgr_mat,cv::COLOR_RGB2BGR);
            return bgr_mat;
        }
        case FSAI_PIXEL_NV12:{
            int raw_height = frame.h * 3 / 2;
            cv::Mat nv12_wrapper(raw_height, frame.w, CV_8UC1,frame.data,
                    static_cast<size_t>(frame.stride));
            cv::Mat bgr_mat;
            cv::cvtColor(nv12_wrapper,bgr_mat,cv::COLOR_YUV2BGR_NV12);
            return bgr_mat;
            }
        default:
            // Unsupported format
            std::cerr << "Warning: Unsupported FsaiPixelFormat: " << frame.fmt << std::endl;
            return cv::Mat();
    }
}

void VisionNode::runProcessingLoop(){
    
    // FIX 2: Add the main "while(running_)" loop
    while(running_){

        // FIX 3: Grab the frame from the camera_ member
        // We use tryGetLatestFrame() for a non-blocking loop.
        std::optional<fsai::vision::FrameRingBuffer::FrameHandle> handle_opt =
            camera_->tryGetLatestFrame();

        // FIX 4: Check if we actually got a frame
        if (!handle_opt) {
            // No frame was available, so we sleep and try again
            std::this_thread::sleep_for(kIdleSleep);
            continue; // Go to the start of the while loop
        }

        // --- Frame is available! ---
        // We can now access the frame data.
        // handle_opt is an optional, so we use -> to access its contents.
        const fsai::types::Frame& left_frame = handle_opt->frame.left;
        const fsai::types::Frame& right_frame = handle_opt->frame.right;

        // You can check frame data (optional)
        if (left_frame.data == nullptr) {
            continue; // Skip this frame
        }

        cv::Mat left_frame_mat = frameToMat(left_frame);
        cv::Mat right_frame_mat = frameToMat(right_frame);
        std::cout << "VisionNode: Running detection on frame " 
                  << handle_opt->frame.t_sync_ns << std::endl;

        fsai::types::Detections new_detections{};
        new_detections.t_ns = handle_opt->frame.t_sync_ns;
        new_detections.n = 0; // Number of cones found
    
        // 1. Object detection logic

        std::vector<FsaiConeDet> detections = detector_->detectCones(left_frame_mat);
        // 2. Loop through the vector and copy into the C-style array
        for (const auto& cone : detections) {
            
            // Safety check: stop if we exceed the array's fixed size
            if (new_detections.n >= 512) {
                std::cerr << "Warning: Detected more than 512 cones. Truncating." << std::endl;
                break;
            }

            // 3. Assign the cone to the next open slot
            new_detections.dets[new_detections.n] = cone;
            
            // 4. Increment the detection counter
            new_detections.n++;
        }

        // 2. Feature matching

        // 3. Stereo triangulation

        // 4. Recovering centre of the cone

        // 5. Recovering global position
    
        // threading fix recommended by Gemini 
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            latest_detections_ = new_detections;
        }
    }
}
}
}//vision