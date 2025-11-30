/**
 * @brief This file handles the joining of the whole vision pipeline from image to ConDet
 * {0: 'blue_cone', 1: 'orange_cone', 2: 'large_orange_cone', 3: 'yellow_cone'}
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
// #include "logging.hpp"
#include <iostream>
#include <chrono>
#include <future>
#include <optional>

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

void VisionNode::reset(){
    std::lock_guard<std::mutex> lock(detection_mutex_);
    fsai::types::Detections empty_msg{};
    empty_msg.n = 0;
    latest_detections_ = empty_msg;
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
std::vector<cv::Point3d> VisionNode::getConeObjectPoints(const int cone_type) {
    std::vector<cv::Point3d> objectPoints;
    // Bottom-Left (Origin)
    double cone_height;
    double cone_width;
    if(cone_type == 2){
        cone_height = LARGE_CONE_HEIGHT;
        cone_width = LARGE_CONE_WIDTH;
    }else{
        cone_height = SMALL_CONE_HEIGHT;
        cone_width = SMALL_CONE_WIDTH;
    }
    objectPoints.clear();
    // 1. Bottom Center (Anchor)
    objectPoints.emplace_back(cone_width / 2.0, 0.0, 0.01); 
    // 2. Top Tip
    objectPoints.emplace_back(cone_width / 2.0, cone_height, 0.0);
    // 3. Bottom Left Corner
    objectPoints.emplace_back(0.0, 0.0, 0.0);
    // 4. Top Left Corner
    objectPoints.emplace_back(0.0, cone_height, 0.0);
    return objectPoints;
}

// 2. Extract the "Pixel" Points (2D) from the Bounding Box
std::vector<cv::Point2d> VisionNode::getConeImagePoints(const types::BoxBound& det) {
    std::vector<cv::Point2d> imagePoints;
    
    // CHANGE: Use double precision for PnP input consistency
    double x = static_cast<double>(det.x);
    double y = static_cast<double>(det.y);
    double w = static_cast<double>(det.w);
    double h = static_cast<double>(det.h);

    // 1. Bottom Center (Anchor)
    imagePoints.emplace_back(x + w / 2.0, y + h); 
    // 2. Top Tip
    imagePoints.emplace_back(x + w / 2.0, y);
    // 3. Bottom Left Corner
    imagePoints.emplace_back(x, y + h);
    // 4. Top Left Corner
    imagePoints.emplace_back(x, y);

    return imagePoints;
}
cv::Rect VisionNode::getPnPROI(const types::BoxBound& det, int im_width, int im_height){
    std::vector<cv::Point3d> objectPoints = getConeObjectPoints(det.side);
    std::vector<cv::Point2d> imagePoints = getConeImagePoints(det);
    
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = cameraParams_.fx;
    K.at<double>(0, 2) = cameraParams_.cx;
    K.at<double>(1, 1) = cameraParams_.fy;
    K.at<double>(1, 2) = cameraParams_.cy;
    
    cv::Mat distCoeffs = cv::Mat::zeros(4,1,CV_64F);
    
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(objectPoints, imagePoints, K, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    
    if (!success) {
        // Fallback: If PnP fails, return a "safe" zero rect or handle upstream
        return cv::Rect(0,0,0,0);
    }
    
    double X_left = tvec.at<double>(0);
    double Z = tvec.at<double>(2); // Depth
    
    double X_right = X_left - BASE_LINE ;
    double u_right = (cameraParams_.fx * X_right / Z) + cameraParams_.cx;
    
    const int H_PADDING = 30; // Pixels
    const int V_PADDING = 20; // Pixels

    int w = det.w + H_PADDING;
    int h = det.h + V_PADDING;
    
    int x = static_cast<int>(u_right - (w / 2.0));
    int y = static_cast<int>(det.y - (V_PADDING / 2.0)); // Keep vertical alignment mostly same

    
    cv::Rect roi(x, y, w, h);
    cv::Rect img_bounds(0, 0, im_width, im_height);
    
    return roi & img_bounds;
}
Eigen::Vector3d VisionNode::getMedianPoint(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty()) return Eigen::Vector3d::Zero();

    size_t n = points.size();
    std::vector<double> xs, ys, zs;
    xs.reserve(n); ys.reserve(n); zs.reserve(n);

    for (const auto& p : points) {
        xs.push_back(p.x());
        ys.push_back(p.y());
        zs.push_back(p.z());
    }

    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());

    auto get_med = [&](const std::vector<double>& v) {
        return (n % 2 == 0) ? (v[n/2 - 1] + v[n/2]) / 2.0 : v[n/2];
    };

    return Eigen::Vector3d(get_med(xs), get_med(ys), get_med(zs));
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
    const double max_depth = 20.0f; 
    const double disparity = feat.x_1 - feat.x_2;

    if (std::abs(disparity) < 1e-6) {
        result.setConstant(std::numeric_limits<double>::infinity());
        return false;
    }

    const double Q = BASE_LINE_ / disparity;
    result.z() = cameraParams_.fx * Q;
    
    if(result.z() >= max_depth || result.z() <= 0.0){ // Added <= 0 check for safety
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
#include <future>   // <--- Added for std::async, std::future
#include <optional> // <--- Added for std::optional

// ... (Your other includes and namespace setup) ...

void VisionNode::runProcessingLoop(){
    
    while(running_){
        if(reset_requested_){
            mapper_.clearMap(); 
            reset_requested_ = false;
        }

        // --- 1. Frame Acquisition and Preprocessing ---
        std::optional<fsai::vision::FrameRingBuffer::FrameHandle> handle_opt =
            camera_->tryGetLatestFrame();

        if (!handle_opt) {
            std::this_thread::sleep_for(kIdleSleep);
            continue;
        }

        auto t_start = std::chrono::high_resolution_clock::now();
        
        if(!intrinsics_set_){
            cameraParams_ = handle_opt->frame.left.K; 
            intrinsics_set_ = true;
        }

        // --- 2. Telemetry and Time ---
        Eigen::Vector2d vehicle_pos {0.0, 0.0};
        double car_yaw_rad = 0.0;
        if(pose_provider_){
            std::pair<Eigen::Vector2d, double> pose = pose_provider_();
            vehicle_pos = pose.first;
            car_yaw_rad = pose.second;
        }

        const uint64_t t_now = handle_opt->frame.t_sync_ns;
        cv::Mat left_mat = frameToMat(handle_opt->frame.left);
        cv::Mat right_mat = frameToMat(handle_opt->frame.right);
        
        // --- 3. YOLO Detection ---
        auto t2 = std::chrono::high_resolution_clock::now();
        std::vector<types::BoxBound> detections = detector_->detectCones(left_mat);
        
        // Render update (Using standard loop debug)
        {
            std::lock_guard<std::mutex> lock(render_mutex_);
            latest_renderable_frame_.image = left_mat; 
            latest_renderable_frame_.boxes = detections;
            latest_renderable_frame_.timestamp_ns = t_now;
            latest_renderable_frame_.valid = true;
        }

        // --- 4. Synchronous Cone Processing Loop (PnP/Match/Median) ---
        auto t3 = std::chrono::high_resolution_clock::now();
        std::vector<FsaiConeDet> raw_global_cones;

        for (int i = 0; i < detections.size(); ++i) {
    const auto& det = detections[i];

    try { // <--- START TRY-CATCH
        
        // A. PnP: Get "Sniper" ROI
        cv::Rect right_roi = getPnPROI(det, right_mat.cols, right_mat.rows);
        if (right_roi.area() <= 0) continue; 

        // B. Match: Targeted SIFT
        cv::Rect left_roi(det.x, det.y, det.w, det.h);
        left_roi = left_roi & cv::Rect(0, 0, left_mat.cols, left_mat.rows);
        
        ConeMatches matches = match_single_cone(left_mat, right_mat, left_roi, right_roi, i, sift_detector_);
        if (matches.matches.empty()) continue; 

        // C. Triangulate
        std::vector<Eigen::Vector3d> points;
        for (const auto& feat : matches.matches) {
            Eigen::Vector3d res;
            if (triangulatePoint(feat, res)) {
                if (res.z() > 0.5 && res.z() < 25.0) {
                    points.push_back(res);
                }
            }
        }
        
        if (points.empty()) continue; 

        // D. Filter (Median)
        Eigen::Vector3d local_cam_pos = getMedianPoint(points);
        if (local_cam_pos.isZero()) continue; 

        // E. Global Transform & Packing (Rest of the loop)
        Eigen::Vector2d vehicle_local_pos;
        vehicle_local_pos.x() = local_cam_pos.z();
        vehicle_local_pos.y() = -local_cam_pos.x();

        Eigen::Isometry2d car_to_global = Eigen::Isometry2d::Identity();
        car_to_global.translation() << vehicle_pos.x(), vehicle_pos.y();
        car_to_global.rotate(Eigen::Rotation2Dd(car_yaw_rad));

        Eigen::Vector2d global_pos = car_to_global * vehicle_local_pos;

        FsaiConeDet glob_det;
        glob_det.x = global_pos.x();
        glob_det.y = 0.0f; 
        glob_det.z = global_pos.y(); 
        glob_det.side = det.side; 
        
        float dist_penalty = 0.02f * (float)local_cam_pos.z();
        glob_det.conf = std::max(0.1f, 0.9f - dist_penalty);

        raw_global_cones.push_back(glob_det);

    } catch (const cv::Exception& e) {
        // CATCH OpenCV exceptions that cause the SIGABRT
        std::cerr << "OpenCV CRITICAL FAIL on cone " << i << ": " << e.what() << std::endl;
        continue;
    } catch (...) {
        // Catch any other unexpected crash
        std::cerr << "Unknown CRITICAL FAIL on cone " << i << std::endl;
        continue;
    }
} // End of serial loop
        
        // --- 5. Map Update ---
        mapper_.update(raw_global_cones, vehicle_pos);

        // --- 6. Serialize Output ---
        fsai::types::Detections out_msg{};
        out_msg.t_ns = t_now;
        out_msg.n = 0;

        for(const auto& c : mapper_.cones){
            if(out_msg.n >= 512) break;
            
            FsaiConeDet d;
            d.x = c.x;
            d.y = c.y; 
            d.z = 0.0f;
            d.side = static_cast<FsaiConeSide>(c.side);
            d.conf = c.conf; 

            out_msg.dets[out_msg.n] = d;
            out_msg.n++;
        }
        detection_buffer_->push(out_msg);
        
        // Old timing logic removed for brevity, can be re-added here.
    }
}
}
}//vision