#include "vision/detection_buffer_registry.hpp"

namespace fsai::vision {

namespace {
    // Internal static variables hidden from other files
    std::mutex& activeMutex() {
        static std::mutex mutex;
        return mutex;
    }

    // Weak pointer: We point to it, but we don't own it. 
    // If VisionNode dies, this becomes invalid automatically.
    std::weak_ptr<DetectionRingBuffer>& activeBufferSlot() {
        static std::weak_ptr<DetectionRingBuffer> buffer;
        return buffer;
    }
}

void setActiveDetectionBuffer(std::shared_ptr<DetectionRingBuffer> buffer) {
    std::lock_guard<std::mutex> lock(activeMutex());
    activeBufferSlot() = buffer; 
}

std::shared_ptr<DetectionRingBuffer> getActiveDetectionBuffer() {
    std::lock_guard<std::mutex> lock(activeMutex());
    // .lock() promotes weak_ptr to shared_ptr. Returns nullptr if the object is gone.
    return activeBufferSlot().lock(); 
}

} // namespace fsai::vision