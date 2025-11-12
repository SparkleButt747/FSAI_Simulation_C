#include "WheelsInfo.h"
#include <stdio.h>

WheelsInfo WheelsInfo_create(float lf_speed,
                             float rf_speed,
                             float lb_speed,
                             float rb_speed,
                             float steering) {
    WheelsInfo info;
    info.lf_speed = lf_speed;
    info.rf_speed = rf_speed;
    info.lb_speed = lb_speed;
    info.rb_speed = rb_speed;
    info.steering = steering;
    return info;
}

WheelsInfo WheelsInfo_default(void) {
    WheelsInfo info;
    info.lf_speed = 0.0f;
    info.rf_speed = 0.0f;
    info.lb_speed = 0.0f;
    info.rb_speed = 0.0f;
    info.steering = 0.0f;
    return info;
}

void WheelsInfo_toString(const WheelsInfo* wheels,
                         char* buffer,
                         int bufferSize) {
    if (!wheels || !buffer || bufferSize <= 0) {
        return;
    }
    snprintf(buffer, bufferSize,
             "LF: %f | RF: %f | LB: %f | RB: %f | Steering: %f",
             wheels->lf_speed,
             wheels->rf_speed,
             wheels->lb_speed,
             wheels->rb_speed,
             wheels->steering);
}
