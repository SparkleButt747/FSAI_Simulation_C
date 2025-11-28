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

#include "vision/detection_buffer_registry.hpp"
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
    constexpr size_t DETECTION_BUFFER_CAPACITY = 10; // Choose a suitable size
    detection_buffer_ = std::make_shared<DetectionsRingBuffer>(DETECTION_BUFFER_CAPACITY);
    sift_detector_ = cv::SIFT::create();

    setActiveDetectionBuffer(detection_buffer_);

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
        std::cout << "VisionNode: " << invalid_dets_ << " were pruned from output" << std::endl;
        std::cout << "VisionNode: found max bbound :" << max_area_ << std::endl;
        std::cout << "VisionNode: found min bbound :" << min_area_ << std::endl;
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
    const double max_depth = 10.0f;
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
    result.z() = cameraParams_.fx * Q;
    if(result.z() >= max_depth){
        invalid_dets_ ++;
        return false;
    }
    // Calculate coordinates directly into the result vector
    result.x() = (feat.x_1 - cameraParams_.cx) * Q;
    
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
        // get car telem at start
        Eigen::Vector2d vehicle_pos {0.0, 0.0};
        double car_yaw_rad = 0.0;

        if(pose_provider_){
            std::pair<Eigen::Vector2d, double> pose = pose_provider_();
            vehicle_pos = pose.first;
            car_yaw_rad = pose.second;
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
        std::vector<types::BoxBound> detections = detector_->detectCones(left_mat);
        for(const auto& det: detections){
            float area = det.w * det.h;
            if(area > max_area_){
                max_area_ = area;
            }else if(area < min_area_){
                min_area_ = area;
            }
        }
        {
            std::lock_guard<std::mutex> lock(render_mutex_);
            latest_renderable_frame_.image = left_mat; // cv::Mat is ref-counted, this is fast
            latest_renderable_frame_.boxes = detections;
            latest_renderable_frame_.timestamp_ns = t_now;
            latest_renderable_frame_.valid = true;
        }

        // 2. Feature matching
        auto t3 = std::chrono::high_resolution_clock::now();
        std::vector<ConeMatches> matched_features;
        try {
            // This is the line causing the crash
            matched_features = match_features_per_cone(left_mat, right_mat, detections,sift_detector_);
        } 
        catch (const cv::Exception& e) {
            // If a crop fails, log it and skip this frame instead of killing the OS process
            fsai::sim::log::LogInfo("[ERROR] OpenCV Exception in match_features: " + std::string(e.what()));
            std::cerr << "Skipping frame due to ROI error." << std::endl;
            continue; // Skip to the next iteration of the while(running_) loop
        }
        // 3. Stereo triangulation
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


        Eigen::Isometry2d car_to_global = Eigen::Isometry2d::Identity();
        car_to_global.translation() << vehicle_pos.x(), vehicle_pos.y();
        car_to_global.rotate(Eigen::Rotation2Dd(car_yaw_rad));

        std::vector<FsaiConeDet> raw_global_cones;

        for(const auto& cone : matched_features){
            
            // 1. Accumulate 3D points
            std::vector<Eigen::Vector3d> points;
            for(const auto& feat : cone.matches){
                Eigen::Vector3d res;
                if(triangulatePoint(feat, res)){
                    // Filter out bad depths immediately
                    // Ignore points behind camera or too close (< 0.5m) or too far (> 20m)
                    if(res.z() > 0.5 && res.z() < 20.0) {
                        points.push_back(res);
                    }
                }
            }

            if(!points.empty()){
                // 2. Explicit Centroid Calculation (Avoids Y/Z confusion)
                // We only care about X (Lateral) and Z (Depth)
                double sum_x = 0.0;
                double sum_z = 0.0;
                
                for(const auto& p : points) {
                    sum_x += p.x();
                    sum_z += p.z();
                }
                
                double avg_x = sum_x / points.size(); // Average Lateral (Camera Right)
                double avg_z = sum_z / points.size(); // Average Depth (Camera Forward)

                // 3. Coordinate Transformation
                // Camera Frame: X=Right, Y=Down, Z=Forward
                // Vehicle Frame: X=Forward, Y=Left, Z=Up
                
                Eigen::Vector2d vehicle_local_pos;
                vehicle_local_pos.x() = avg_z;   // Camera Forward (Z) -> Vehicle Forward (X)
                vehicle_local_pos.y() = -avg_x;  // Camera Right (X)   -> Vehicle Left (Y) (Note the negative)

                // SANITY CHECK: Ignore cones that resolved to be "behind" the car
                // This handles edge cases where calibration noise produces negative Z
                if (vehicle_local_pos.x() <= 0.1) continue; 

                // 4. Global Transform
                Eigen::Vector2d global_pos = car_to_global * vehicle_local_pos;
                
                FsaiConeDet glob_det;
                glob_det.x = global_pos.x();
                glob_det.z = global_pos.y(); // Map Global Y -> Struct Z
                glob_det.y = 0.0f;           // Elevation
                
                glob_det.side = cone.side;
                
                // --- Confidence Calculation ---
                // Heuristic: Confidence drops with distance
                // Base 0.9, minus 0.02 for every meter away
                float dist_penalty = 0.02f * (float)avg_z;
                glob_det.conf = std::max(0.1f, 0.9f - dist_penalty);

                raw_global_cones.push_back(glob_det);
                
                // [Optional Debugging]
                // std::cout << "Cone Local: " << vehicle_local_pos.x() << "m Fwd, " 
                //           << vehicle_local_pos.y() << "m Left" << std::endl;
            }
        }

        // 7. Map Update
        mapper_.update(raw_global_cones, vehicle_pos);

        // 7. Serialize Output
        fsai::types::Detections out_msg{};
        out_msg.t_ns = handle_opt->frame.t_sync_ns;
        out_msg.n = 0;

        for(const auto& c : mapper_.cones){
            if(out_msg.n >= 512) break;
            
            FsaiConeDet d;
            d.x = c.x;
            d.y = c.y; // Map Y -> Output Z
            d.z = 0.0f;
            d.side = static_cast<FsaiConeSide>(c.side);
            d.conf = c.conf; 

            out_msg.dets[out_msg.n] = d;
            out_msg.n++;
        }
        detection_buffer_->push(out_msg);
        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        fsai::sim::log::LogInfo("[VISION]: DETECTED " + std::to_string(detections.size()));
        fsai::sim::log::LogInfo("[VISION]: MAPPED " + std::to_string(out_msg.n) + " CONES IN " + std::to_string(duration_ms) + " ms");
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
        fsai::sim::log::LogInfo("[VISION/CTRL] SIZE OF BUFFER " + std::to_string(detection_buffer_->size()));
    }
}
}
}//vision