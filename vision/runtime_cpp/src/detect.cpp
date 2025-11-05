#include "common/include/common/types.h"
#include "vision/detect.hpp"

#include <onnxruntime_cxx_api.h> 
#include <opencv2/opencv.hpp> 
#include <iostream>
#include <vector>
#include <string>


//{0: 'blue_cone', 1: 'orange_cone', 2: 'large_orange_cone', 3: 'yellow_cone'}
namespace fsai{
namespace vision{
const int HEIGHT = 640;
const int WIDTH = 640;
const float THRESHOLD = 0.8f;
const float IOU_OVERLAP = 0.5f;
const int CONFIDENCE_STARTING_INDEX = 4;

[[deprecated("Replaced by ConeDetector class to improve effecient")]]
std::vector<BoxBound> detect_cones(cv::Mat image){
    std::cout << "STARTING YOLO DETECTION" << std::endl;


    //load model using onnx
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLO_DETECTOR");
    Ort::SessionOptions session_options;
    const char* model_path  =  "../../model/cone_model.onnx";
    Ort::Session session(env,model_path,session_options);

    std::cout << "--- ONNX model loaded ---" << std::endl;

    cv::Mat resized_image;
    cv::resize(image,resized_image, cv::Size(HEIGHT,WIDTH));
    std::vector<float> input_tensor_values(1*3*HEIGHT*WIDTH);

    float* data_ptr = input_tensor_values.data();

    for(int c = 0; c <3;c++){ //channels
        for(int h = 0; h < HEIGHT; h++){
            for(int w = 0; w < WIDTH; w++){

                int channel_idx = (c == 0) ? 2 : ((c == 2) ? 0 : 1);
            
                // Calculate the position in flat CHW array
                int data_index = c * (640 * 640) + h * 640 + w;
                
                // Normalize and assign
                data_ptr[data_index] = (float)resized_image.at<cv::Vec3b>(h, w)[channel_idx] / 255.0f;
            }
        
        }
    }

    std::cout << "Image preprocessing complete" << std::endl;

    //make detections using model

    const char* input_node_names[] = {"images"};
    const char* output_nodes_names[] = {"output0"};

    int64_t input_shape[] = {1,3,HEIGHT,WIDTH};

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, 
        input_tensor_values.data(), 
        input_tensor_values.size(), 
        input_shape, 
        4 // 4 dimensions
    );

    auto output_tensors = session.Run(
        Ort::RunOptions{nullptr},
        input_node_names, &input_tensor, 1,
        output_nodes_names, 1
    );

    std::cout << "--- Detections made, starting post-proc ---" << std::endl;

    Ort::Value& output_tensor = output_tensors[0];

    float* output_data = output_tensor.GetTensorMutableData<float>();

    auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
    long num_proposals = output_shape[2];
    long proposal_size = output_shape[1];

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    float x_scale = (float)image.cols / 640.0f;
    float y_scale = (float)image.rows / 640.0f;

    // Dynamically get the number of classes
    const int num_classes = (int)proposal_size - 4;
    const float* x_data = output_data; 
    const float* y_data = output_data + num_proposals;
    const float* w_data = output_data + (2 * num_proposals);
    const float* h_data = output_data + (3 * num_proposals);
    // Class data starts after the 4 box rows
    const float* class_data_start = output_data + (4 * num_proposals);
    
    for(long i = 0; i < num_proposals; i++){

        // float* dectection_data = output_data + i * proposal_size;
        //unpack data
        float max_conf = 0;
        int class_id = -1;

        for(int j = 0; j < num_classes; j++){
            // Get the score for this class (j) and this proposal (i)
            float score = (class_data_start + j * num_proposals)[i];
            
            if(score > max_conf){ 
                max_conf = score;
                class_id = j; // Class ID will be 0, 1, 2, or 3
            }
        }

        if(max_conf > THRESHOLD){
            //add to rectangles
            int cx = x_data[i];
            int cy = y_data[i];
            int w = w_data[i];
            int h = h_data[i];

            int x_min = static_cast<int>((cx - w / 2.0) * x_scale);
            int y_min = static_cast<int>((cy - h / 2.0) * y_scale);
            int box_width = static_cast<int>(w * x_scale);
            int box_height = static_cast<int>(h * y_scale);
            
            boxes.push_back(cv::Rect(x_min,y_min,box_width,box_height));

            confidences.push_back(max_conf);
            class_ids.push_back(class_id);
        }
    }

    // clean up noise using NMS
    std::vector<int> nms_res;
    cv::dnn::NMSBoxes(
        boxes,
        confidences,
        THRESHOLD,
        IOU_OVERLAP,
        nms_res);
    std::vector<BoxBound> bounds;
    for (int index : nms_res){

        //create instances of detections
        cv::Rect& final_box = boxes[index]; 
        int class_id = class_ids[index];
        fsai::types::ConeSide side;
        if(class_id == 0){
            side = FSAI_CONE_LEFT;
        }else{
            side = FSAI_CONE_RIGHT;
        }   
        BoxBound bound = {
            final_box.x,
            final_box.y,
            (float)final_box.width,
            (float)final_box.height,
            confidences[index],
            side // Using this as a placeholder
        };
        bounds.push_back(bound);
    }

    return bounds;
    
}

ConeDetector::ConeDetector(const std::string& path_to_model){
    if(path_to_model.empty()){
        std::cerr << "VisionNode: ConeDetector: path to model not supplied" << std::endl;
        throw std::invalid_argument("Path to model was null");
    }
    try{
        std::cout << "VisionNode: ConeDetector: Loading model" << std::endl;

        Ort::SessionOptions session_options;
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ConeDetector");
        session_ = std::make_unique<Ort::Session>(*env_, path_to_model.c_str(), session_options);

    }catch(const Ort::Exception& e){
        std::cerr << "VisionNode: ConeDetector: Error whilst loading onnx model" << e.what() << std::endl;
    }
}

ConeDetector::~ConeDetector() = default;

std::vector<fsai::types::ConeDet> ConeDetector::detectCones(const cv::Mat& left_frame){
    // preprocess frames to be 640*640px
    cv::Mat resized_image;
    cv::resize(left_frame,resized_image, cv::Size(HEIGHT,WIDTH));
    std::vector<float> input_tensor_values(1*3*HEIGHT*WIDTH);

    float* data_ptr = input_tensor_values.data();

    for(int c = 0; c <3;c++){ //channels
        for(int h = 0; h < HEIGHT; h++){
            for(int w = 0; w < WIDTH; w++){

                int channel_idx = (c == 0) ? 2 : ((c == 2) ? 0 : 1);
            
                // Calculate the position in flat CHW array
                int data_index = c * (640 * 640) + h * 640 + w;
                
                // Normalize and assign
                data_ptr[data_index] = (float)resized_image.at<cv::Vec3b>(h, w)[channel_idx] / 255.0f;
            }
        
        }
    }

    std::cout << "Image preprocessing complete" << std::endl;

    //make detections using model

    const char* input_node_names[] = {"images"};
    const char* output_nodes_names[] = {"output0"};

    int64_t input_shape[] = {1,3,HEIGHT,WIDTH};

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, 
        input_tensor_values.data(), 
        input_tensor_values.size(), 
        input_shape, 
        4 // 4 dimensions
    );

    auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
        input_node_names, &input_tensor, 1,
        output_nodes_names, 1
    );

    Ort::Value& output_tensor = output_tensors[0];

    float* output_data = output_tensor.GetTensorMutableData<float>();

    auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
    long num_proposals = output_shape[2];
    long proposal_size = output_shape[1];

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    float x_scale = (float)left_frame.cols / 640.0f;
    float y_scale = (float)left_frame.rows / 640.0f;

    // Dynamically get the number of classes
    const int num_classes = (int)proposal_size - 4;
    const float* x_data = output_data; 
    const float* y_data = output_data + num_proposals;
    const float* w_data = output_data + (2 * num_proposals);
    const float* h_data = output_data + (3 * num_proposals);
    // Class data starts after the 4 box rows
    const float* class_data_start = output_data + (4 * num_proposals);
    for(long i = 0; i < num_proposals; i++){

        // float* dectection_data = output_data + i * proposal_size;
        //unpack data
        float max_conf = 0;
        int class_id = -1;

        for(int j = 0; j < num_classes; j++){
            // Get the score for this class (j) and this proposal (i)
            float score = (class_data_start + j * num_proposals)[i];
            
            if(score > max_conf){ 
                max_conf = score;
                class_id = j; // Class ID will be 0, 1, 2, or 3
            }
        }

        if(max_conf > THRESHOLD){
            //add to rectangles
            int cx = x_data[i];
            int cy = y_data[i];
            int w = w_data[i];
            int h = h_data[i];

            int x_min = static_cast<int>((cx - w / 2.0) * x_scale);
            int y_min = static_cast<int>((cy - h / 2.0) * y_scale);
            int box_width = static_cast<int>(w * x_scale);
            int box_height = static_cast<int>(h * y_scale);
            
            boxes.push_back(cv::Rect(x_min,y_min,box_width,box_height));

            confidences.push_back(max_conf);
            class_ids.push_back(class_id);
        }
    }
    std::vector<int> nms_res;
    cv::dnn::NMSBoxes(
        boxes,
        confidences,
        THRESHOLD,
        IOU_OVERLAP,
        nms_res);
    std::vector<FsaiConeDet> detections;
    for (int index : nms_res){

        //create instances of detections
        cv::Rect& final_box = boxes[index]; 
        int class_id = class_ids[index];
        fsai::types::ConeSide side;
        if(class_id == 0){
            side = FSAI_CONE_LEFT;
        }else{
            side = FSAI_CONE_RIGHT;
        }   
        BoxBound bound = {
            final_box.x,
            final_box.y,
            (float)final_box.width,
            (float)final_box.height,
            confidences[index],
            side // Using this as a placeholder
        };
        detections.push_back(processDetection(bound));
    }
    return detections;
}

fsai::types::ConeDet ConeDetector::processDetection(const BoxBound& box_bound){
    // create a new ConeDet
    if(box_bound.w <= 0 || box_bound.h <= 0 || box_bound.x <= 0 || box_bound.y <= 0){
        return FsaiConeDet();
    }
   
    FsaiConeDet cone = {
        {(float)box_bound.x,(float)box_bound.y,-1.0f},
        box_bound.side,
        box_bound.conf
    };
    return cone;
}

}
}
