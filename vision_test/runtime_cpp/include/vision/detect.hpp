#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/include/common/types.h"

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
  int gradient_threshold = 60;
  int min_component_pixels = 48;
  float confidence_normalizer = 256.0f;
};

class EdgeDetector {
 public:
  explicit EdgeDetector(EdgeDetectorOptions options = {});

  std::vector<BoxBound> process(const FsaiStereoFrame& frame) const;

 private:
  EdgeDetectorOptions options_{};
};

}  // namespace fsai::vision

