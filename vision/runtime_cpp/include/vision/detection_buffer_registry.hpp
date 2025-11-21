// detection_buffer_registry.hpp
#pragma once

#include <memory>
#include <mutex>
#include "vision/shared_ring_buffer.hpp"
#include "common/include/common/types.h" // Where fsai::types::Detections is defined

namespace fsai::vision {

// 1. Define the specific type we are making global
using DetectionRingBuffer = fsai::vision::GenericRingBuffer<fsai::types::Detections>;

// 2. Global Accessor Declarations
// Returns a pointer to the currently active buffer (or null if none exists)
std::shared_ptr<DetectionRingBuffer> getActiveDetectionBuffer();

// Sets the currently active buffer
void setActiveDetectionBuffer(std::shared_ptr<DetectionRingBuffer> buffer);

} // namespace fsai::vision