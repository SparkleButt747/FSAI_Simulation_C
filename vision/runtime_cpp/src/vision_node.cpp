/**
 * @brief This file handles the joining of the whole vision pipeline from image to ConDet
 * TO-DO:
 * Connect to centroid recovery
 * Calculate glob coordinate
 */
#include "vision/vision_node.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>

#include "detect.hpp"
#include "sim_camera.hpp"
#include "features.hpp"
#include "centroid.hpp"

#include "common/include/common/types.h"
#include "logging.hpp"
#include <iostream>
#include <chrono>

const char* PATH_TO_MODEL = "../vision/models/cone_model.onnx";
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

RenderableFrame VisionNode::getRenderableFrame(){
            std::lock_guard<std::mutex> lock(render_mutex_);
            // Return a deep copy to ensure thread safety when the render thread uses it
            RenderableFrame ret;
            ret.image = latest_renderable_frame_.image.clone();
            ret.boxes = latest_renderable_frame_.boxes;
            ret.timestamp_ns = latest_renderable_frame_.timestamp_ns;
            ret.valid = latest_renderable_frame_.valid;
            return ret;
}

/**
 * @brief Applies stereo triangulation to a pair of points
 * @param Feature point the matched feature
 */
inline bool VisionNode::triangulatePoint(const Feature& feat, Eigen::Vector3d& result){
    const double disparity = feat.x_1 - feat.x_2;

    // Check for near-zero disparity to avoid division by zero
    if (std::abs(disparity) < 1e-6) {
        // Optimization: Use standard generic infinity if Eigen supports it, 
        // or keep your specific error handling here.
        result.setConstant(std::numeric_limits<double>::infinity());
        return false;
    }

    // Precompute common factor Q = Baseline / Disparity
    // This saves multiple multiplications and divisions later.
    const double Q = BASE_LINE_ / disparity;

    // Calculate coordinates directly into the result vector
    result.x() = (feat.x_1 - cameraParams_.cx) * Q;
    
    result.z() = cameraParams_.fx * Q;

    result.y() = (feat.y_1 - cameraParams_.cy) * result.z() / cameraParams_.fy;

    return true;
}

std::optional<fsai::types::Detections> VisionNode::getLatestDetections(){
    if(latest_detections_->n >= 0){
        std::lock_guard<std::mutex> lock(detection_mutex_);
        return latest_detections_;
    }
}

void VisionNode::runProcessingLoop(){
    
    // FIX 2: Add the main "while(running_)" loop
    
    while(running_){

        // FIX 3: Grab the frame from the camera_ member
        // We use tryGetLatestFrame() for a non-blocking loop.
        std::optional<fsai::vision::FrameRingBuffer::FrameHandle> handle_opt =
            camera_->tryGetLatestFrame();

        if (!handle_opt) {
            // No frame was available, so sleep and try again
            std::this_thread::sleep_for(kIdleSleep);
            continue;
        }
        auto t_start = std::chrono::high_resolution_clock::now();
        if(!intrinsics_set_){
            cameraParams_ = handle_opt->frame.left.K; //params will be the same for both frames
            intrinsics_set_ = true;
        }
        // --- Frame is available! ---
        const uint64_t t_now = handle_opt->frame.t_sync_ns;
        // 1. Convert to Mat (local variable only for now)
        auto t1 = std::chrono::high_resolution_clock::now();
        cv::Mat left_mat = frameToMat(handle_opt->frame.left);
        cv::Mat right_mat = frameToMat(handle_opt->frame.right);
        
        fsai::types::Detections new_detections{};
        new_detections.t_ns = handle_opt->frame.t_sync_ns;
        new_detections.n = 0; // Number of cones found
    
        // 1. Object detection logic
        auto t2 = std::chrono::high_resolution_clock::now();
        std::vector<BoxBound> detections = detector_->detectCones(left_mat);
        {
            std::lock_guard<std::mutex> lock(render_mutex_);
            latest_renderable_frame_.image = left_mat; // cv::Mat is ref-counted, this is fast
            latest_renderable_frame_.boxes = detections;
            latest_renderable_frame_.timestamp_ns = t_now;
            latest_renderable_frame_.valid = true;
        }

        // 2. Loop through the vector and copy into the C-style array
        // 2. Feature matching
        auto t3 = std::chrono::high_resolution_clock::now();
        std::vector<ConeMatches> matched_features = match_features_per_cone(left_mat,right_mat,detections);
        // 3. Stereo triangulation
        // convert the features to 2D point struct
        auto t4 = std::chrono::high_resolution_clock::now();
        std::vector<ConeCluster> cone_cluster;
        
        std::vector<Eigen::Vector2d> local_centres;

        for(const auto& cone:matched_features){
            //iterate over each feature for every cone
            ConeCluster cluster;
            cluster.coneId = cone.cone_index;
            for(const auto& feat: cone.matches ){
                //determine depth for each match and update cone cluster
                Eigen::Vector3d res;
                if(triangulatePoint(feat,res)){
                    cluster.points.push_back(res);
                }
            }
            if(!cluster.points.empty()){
                // refactored to run the centroid recovery here to save space
                // find 2d centre coord and add to intermittent vector
                Eigen::Vector2d center = Centroid::centroid_linear(cluster.points);
                if(!center.isZero()){
                    local_centres.push_back(center);
                }
            }
        } 
        // 5. Recovering global position
        //apply rigid body transformation and translation
        Eigen::Vector2d vehicle_pos {0.0,0.0};
        double car_yaw_rad = 0.0f;
        if(pose_provider_){
            auto pose = pose_provider_();
            vehicle_pos = pose.first;
            car_yaw_rad = pose.second;
        }
        // std::cout << "Vision Received Car Pos:" << vehicle_pos.x() << vehicle_pos.y() << std::endl;

        Eigen::Isometry2d car_to_global = Eigen::Isometry2d::Identity();
        car_to_global.translation() << vehicle_pos.x(), vehicle_pos.y();
        car_to_global.rotate(Eigen::Rotation2Dd(car_yaw_rad));

        for(const auto& center : local_centres) {
            FsaiConeDet glob_det;
            // 3. CONVERT Camera Frame (Z-forward, X-right) to Vehicle Frame (X-forward, Y-left)
            Eigen::Vector2d vehicle_pos;
            vehicle_pos.x() = center.y();  // Forward
            vehicle_pos.y() = -center.x(); // Left (negative right)

            // Apply Global Transform
            Eigen::Vector2d global_pos = car_to_global * vehicle_pos;
            glob_det.x = global_pos.x();
            glob_det.y = global_pos.y();
            glob_det.z = 0.0f;
            new_detections.dets[new_detections.n] = glob_det;
            new_detections.n++;
        }
        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        fsai::sim::log::LogInfo("[VISION]: MAPPED " + std::to_string(new_detections.n) + " CONES IN " + std::to_string(duration_ms) + " ms");
        auto ms_conv = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto ms_yolo = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto ms_match = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        auto ms_rest = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t4).count();
        auto ms_total = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

        fsai::sim::log::LogInfo(
            "[PROFILE] Total: " + std::to_string(ms_total) + "ms | " +
            "Conv: " + std::to_string(ms_conv) + " | " +
            "YOLO: " + std::to_string(ms_yolo) + " | " +
            "Match: " + std::to_string(ms_match) + " | " +
            "Post: " + std::to_string(ms_rest)
        );
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            latest_detections_ = new_detections;
        }
    }
}
}
}//vision