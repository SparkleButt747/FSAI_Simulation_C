#pragma once

#include <cstdint>

#include "common/types.h"
#include "Vector.h"
#include "sim/integration/path_truth.hpp"

namespace fsai {
namespace integration {

enum AdapterStatusBits : uint32_t {
    AD_STATUS_OK = 0,
    AD_STATUS_NO_LEFT = 1u << 0,
    AD_STATUS_NO_RIGHT = 1u << 1,
    AD_STATUS_FALLBACK_PATH = 1u << 2,
};

struct AdapterOutput {
    int count{0};
    uint32_t status{AD_STATUS_OK};
};

AdapterOutput Adapter_BuildCheckpoints(const FsaiDetections* dets_or_null,
                                       const PathMeta* path_or_null,
                                       Vector3* checkpointPositions,
                                       int maxN,
                                       float sample_m,
                                       float horizon_m);

}  // namespace integration
}  // namespace fsai

