#include "vision/vision_node.hpp" // Your main API header
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "[TestApp] Starting VisionNode test..." << std::endl;

    try {
        // 1. Instantiate the node
        // This tests:
        // - Constructor linking (vision_node.cpp)
        // - SimCamera linking (sim_camera.cpp)
        // - ConeDetector linking (detect.cpp)
        // - ONNX model loading
        std::cout << "[TestApp] Creating VisionNode..." << std::endl;
        fsai::vision::VisionNode vision_node;
        std::cout << "[TestApp] VisionNode created successfully." << std::endl;

        // 2. Start the processing thread
        // This tests:
        // - start() and stop() linking
        // - std::thread linking (pthreads)
        vision_node.start();
        std::cout << "[TestApp] VisionNode thread started." << std::endl;

        // 3. Let it run for a few seconds
        std::cout << "[TestApp] Running for 5 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 4. Stop the node
        std::cout << "[TestApp] Stopping VisionNode..." << std::endl;
        vision_node.stop();
        std::cout << "[TestApp] VisionNode stopped." << std::endl;

    } catch (const std::exception& e) {
        // If the constructor fails (e.g., can't find model), this will catch it
        std::cerr << "[TestApp] FATAL ERROR: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "[TestApp] FATAL ERROR: An unknown exception occurred." << std::endl;
        return 1;
    }

    std::cout << "[TestApp] Test completed successfully." << std::endl;
    return 0;
}