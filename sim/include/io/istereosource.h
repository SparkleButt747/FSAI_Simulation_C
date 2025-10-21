#pragma once

#include "common/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FsaiStereoSource {
    void* impl;
    int (*start)(void* impl);
    int (*grab)(void* impl, FsaiStereoFrame* out_frame);
    void (*stop)(void* impl);
} FsaiStereoSource;

#ifdef __cplusplus
}
#endif
