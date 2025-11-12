#include "common/include/common/types.h"
#include "detect.hpp"

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

ConeDetector::ConeDetector(const std::string& path_to_model){
    if(path_to_model.empty()){
        std::cerr << "VisionNode: ConeDetector: path to model not supplied" << std::endl;
        throw std::invalid_argument("Path to model was null");
    }
    try{
        std::cout << "VisionNode: ConeDetector: Loading model" << std::endl;
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ConeDetector");

        Ort::SessionOptions session_options;
        try {
            OrtCUDAProviderOptions cuda_options;
            cuda_options.device_id = 0; // Use GPU 0 (your RTX 3060)
            // Optional: tune these if needed later
            // cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive; 
            // cuda_options.arena_extend_strategy = 0;
            // cuda_options.do_copy_in_default_stream = 1;
            
            session_options.AppendExecutionProvider_CUDA(cuda_options);
            std::cout << "VisionNode: CUDA provider appended successfully." << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "WARNING: Failed to append CUDA provider: " << ex.what() << std::endl;
            std::cerr << "Falling back to CPU." << std::endl;
            // Optionally re-throw if CUDA is strictly required
        }
        // --- ENABLE CUDA END ---

        // Optimization usually helps performance regardless of provider
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(*env_, path_to_model.c_str(), session_options);

    }catch(const Ort::Exception& e){
        std::cerr << "VisionNode: ConeDetector: Error whilst loading onnx model " << e.what() << std::endl;
        throw std::invalid_argument("[Cone Detector]Path to model could not be loaded");
    }
}

ConeDetector::~ConeDetector() = default;

std::vector<BoxBound>ConeDetector::detectCones(const cv::Mat& left_frame){
    // preprocess frames to be 640*640px
    cv::Mat blob = cv::dnn::blobFromImage(left_frame, 1.0/255.0, 
                                          cv::Size(WIDTH, HEIGHT), 
                                          cv::Scalar(), true, false);

    // 2. Wrap the blob data in an ORT tensor (zero-copy if possible)
    // blob is already in NCHW float32 format thanks to blobFromImage
    
    int64_t input_shape[] = {1, 3, HEIGHT, WIDTH};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    // blob.ptr<float>() gives direct access to the prepared data
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, 
        blob.ptr<float>(), 
        blob.total(), 
        input_shape, 
        4
    );
    const char* input_node_names[] = {"images"};
    const char* output_nodes_names[] = {"output0"};
    // 3. Run inference
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
    std::vector<BoxBound> detections;
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
        detections.push_back(bound);
    }
    return detections;
}

fsai::types::ConeDet ConeDetector::processDetection(const BoxBound& box_bound){
    // create a new ConeDet
    if(box_bound.w <= 0 || box_bound.h <= 0 || box_bound.x <= 0 || box_bound.y <= 0){
        return FsaiConeDet();
    }
   
    fsai::types::ConeDet cone = {
        static_cast<float>(box_bound.x),static_cast<float>(box_bound.y),-1.0f,
        box_bound.side,
        box_bound.conf
    };
    return cone;
}

}
}
