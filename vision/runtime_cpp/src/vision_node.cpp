#include "vision/api.h"

int Vision_Init(const char* /*config_yaml*/) {
    return 0;
}

int Vision_ProcessStereo(const FsaiStereoFrame* /*frame*/, FsaiDetections* out_detections) {
    if (out_detections) {
        out_detections->n = 0;
    }
    return 1;
}

void Vision_Shutdown(void) {}
