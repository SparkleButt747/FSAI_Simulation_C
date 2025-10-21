#pragma once

#include "common/types.h"

#ifdef __cplusplus
extern "C" {
#endif

int Vision_Init(const char* config_yaml);
int Vision_ProcessStereo(const FsaiStereoFrame* frame, FsaiDetections* out_detections);
void Vision_Shutdown(void);

#ifdef __cplusplus
}
#endif
