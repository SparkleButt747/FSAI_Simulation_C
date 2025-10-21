#include "control/api.h"

#include "control/clc_controller.hpp"
#include "fsai_clock.h"

struct ControlContext {
    CarController controller;
};

static const char kDefaultVehicleConfig[] = "../sim/src/vehicle/Configs/configDry.yaml";

ControlContext* Control_Init(const char* config_yaml) {
    auto* ctx = new ControlContext();
    const char* vehicle_config = (config_yaml && config_yaml[0] != '\0') ? config_yaml : kDefaultVehicleConfig;
    CarController_Init(&ctx->controller, vehicle_config, nullptr);
    return ctx;
}

int Control_Tick(ControlContext* ctx, const FsaiVehicleState* /*state*/, const FsaiDetections* /*detections*/, FsaiControlCmd* out_cmd) {
    if (!ctx || !out_cmd) {
        return 0;
    }
    const uint64_t now_ns = fsai_clock_now();
    *out_cmd = CarController_Update(&ctx->controller, 0.0, now_ns);
    return 1;
}

void Control_Shutdown(ControlContext* ctx) {
    delete ctx;
}
