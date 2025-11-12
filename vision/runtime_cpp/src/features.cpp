#include <iostream>
#include "features.hpp"
#include "vector"
#include "detect.hpp"
#include <unordered_map>
#include <opencv2/opencv.hpp>


// high level overview 
// we need function to take left and right frames and bounding box and return xy coordinates of matched features for each bounding box

namespace fsai {
    namespace vision{
        cv::Mat extract_boundimg(cv::Mat left_frame, BoxBound box_bound){
            // extracts the bounding box image for a single bounding box object
            cv::Rect roi(box_bound.x, box_bound.y, box_bound.w, box_bound.h);   // define region of interest for extraction

            //Clip the roi to be in frame dimensions
            roi &= cv::Rect(0,0, left_frame.cols, left_frame.rows);

            if (roi.width <= 0 || roi.height <= 0) {
                std::cerr << "Warning: Out of bands ROI requested." << std::endl;
                return cv::Mat();
            }
            cv::Mat box_boundimg = left_frame(roi);                               // extracts the region we want as a reference (we never modify it anyways, no need to clone)
            return box_boundimg;                                                // returns a reference to the part of the frame we want

        }

        std::vector<pseudofeature> extract_features(cv::Mat frame, cv::Ptr<cv::ORB> orb){
            // extract all features from a frame. If using for left frame, pass in results from extract_boundimg
            // each pseudofeature has xy coordinates in coordinate system of right frame, and a descriptor matrix used for ORB or SIFT

            //Empty frame check
            if (frame.empty()) {
                std::cerr << "Warning: Empty frame received" << std::endl;
                return {};
            }

            // declare keypoints and descriptors matrix
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;

            // detect and compute
            orb->detect(frame, keypoints);
            orb->compute(frame, keypoints, descriptors);

            std::vector<pseudofeature> results;

            for (int i = 0; i < keypoints.size(); i++){
                cv::KeyPoint keypoint_i = keypoints[i];
                cv::Mat descriptor_i = descriptors.row(i);

                //keypoint has attribute pt, which is (x,y)
                int x = keypoint_i.pt.x;
                int y = keypoint_i.pt.y;

                // initialise new pseudofeature and append
                pseudofeature pseudofeature_i{(double)x,(double)y,descriptor_i};
                results.push_back(pseudofeature_i);
            }

            return results; // should be a vector of pseudofeatures, e.g. [(x,y,descriptor), (x,y,descriptor), ...]
        }

        std::vector<feature> pair_features(std::vector<pseudofeature> left_features, std::vector<pseudofeature> right_features){
            // use epipolar constraint to search along epipolar line for each left feature to find matching right feature
            // return an "array" of (left_feature, right_feature) to pass for coordinate calculation


            //define output
            std::vector<feature> results;

            //create dictionary for y-hash for quick lookup
            std::unordered_map<int, std::vector<pseudofeature>> y_mappings = {};

            //add right_features to hash table
            for (int i = 0; i < right_features.size(); i++){
                pseudofeature right_feature_i = right_features[i];
                int y = right_feature_i.y;
                y_mappings[y].push_back(right_feature_i);
            }

            //define how much to search up and down, e.g. +-1 pixel if it is 1
            int epipolar_tolerance = 1;

            //try to find matching features
            for (int i = 0; i < left_features.size(); i++){
                pseudofeature left_feature_i = left_features[i];
                int y = left_feature_i.y;
                int x = left_feature_i.x;

                //temp variable to hold possible matches
                std::vector<pseudofeature> possible_matches;

                // look for possible matches within tolerance
                for (int j = y-epipolar_tolerance; j < y+epipolar_tolerance; j++){
                    std::vector<pseudofeature> fetched_features = y_mappings[j];
                    possible_matches.insert(possible_matches.end(), fetched_features.begin(), fetched_features.end());
                }

                int best_x, best_y;
                int min = -1;

                // for each possible match, compute hamming distance and find lowest
                for (int j = 0; j < possible_matches.size(); j++){
                    pseudofeature right_feature_j = possible_matches[j];
                    int right_x = right_feature_j.x;
                    int right_y = right_feature_j.y;
                    cv::Mat left_descriptor= left_feature_i.descriptors, right_descriptor = right_feature_j.descriptors;

                    int hamming_distance = cv::norm(left_descriptor, right_descriptor, cv::NORM_HAMMING);

                    if (min == -1){
                        min = hamming_distance;
                        best_x = right_x, best_y = right_y;
                    }
                    else if (hamming_distance < min){
                        min = hamming_distance;
                        best_x = right_x, best_y = right_y;
                    }
                    else{
                        // do nothing
                    }
                }

                feature result_feature{(double)x,(double)y,(double)best_x,(double)best_y};
                results.push_back(result_feature);
            }
            return results;
        }


        std::vector<feature> match_features(cv::Mat left_frame, cv::Mat right_frame,std::vector<BoxBound> box_bounds){

            //create orb object to feed into extraction functions
            cv::Ptr<cv::ORB> orb = cv::ORB::create();

            //create variable to hold left_features
            std::vector<pseudofeature> left_features;
            cv::Mat extracted_image;
            std::vector<pseudofeature> extracted_features;
            // go over each bounding box
            for (int i = 0; i < box_bounds.size(); i++){
                fsai::vision::BoxBound box_i = box_bounds[i];
                int x = box_i.x;
                int y = box_i.y;

                //extract bounding box image and its features
                extracted_image = extract_boundimg(left_frame, box_i);
                extracted_features = extract_features(extracted_image, orb);

                //make it relative to entire left frame instead of bounding box
                for (int j = 0; j < extracted_features.size();j++){
                    pseudofeature feature_i = extracted_features[i];
                    feature_i.x = feature_i.x + x;
                    feature_i.y = feature_i.y + y;
                    left_features.push_back(feature_i);
                }
            }

            //create variable to hold right_features
            std::vector<pseudofeature> right_features;
            right_features = extract_features(right_frame, orb);

            std::vector<feature> results;
            results = pair_features(left_features, right_features);

            return results;
        }

    }
}


int main(){
    // test the feature matching based on left frame and right frame
    std::cout << "Build Success!!!!";
    return 0;
}
