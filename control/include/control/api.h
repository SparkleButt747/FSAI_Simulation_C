#pragma once

#include "common/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ControlContext ControlContext;

ControlContext* Control_Init(const char* config_yaml);
int Control_Tick(ControlContext* ctx, const FsaiVehicleState* state, const FsaiDetections* detections, FsaiControlCmd* out_cmd);
void Control_Shutdown(ControlContext* ctx);

#ifdef __cplusplus
}
#endif
