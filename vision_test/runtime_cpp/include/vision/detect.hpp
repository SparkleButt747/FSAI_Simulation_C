#pragma once

#include "common/types.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fsai::vision {

struct BoxBound {
  int x = 0;
  int y = 0;
  float w = 0.0f;
  float h = 0.0f;
  float conf = 0.0f;
  fsai::types::ConeSide side = fsai::types::ConeSide::FSAI_CONE_UNKNOWN;
};

struct EdgeDetectorOptions {
  int gradient_threshold = 32;
  int min_component_pixels = 50;
  float confidence_normalizer = 128.0f;
};

class EdgeDetector {
 public:
  EdgeDetector() = default;
  explicit EdgeDetector(EdgeDetectorOptions options);

  std::vector<BoxBound> process(const FsaiStereoFrame& frame) const;

 private:
  EdgeDetectorOptions options_{};
};

}  // namespace fsai::vision

