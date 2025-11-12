#ifndef WHEELSINFO_H
#define WHEELSINFO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simple POD for wheel speeds and steering angle.
 *
 * Speeds are typically [rpm] or [m/s] depending on your usage.
 * Steering in [rad].
 */
typedef struct {
    float lf_speed;   // Left-Front wheel
    float rf_speed;   // Right-Front wheel
    float lb_speed;   // Left-Rear wheel
    float rb_speed;   // Right-Rear wheel
    float steering;   // Front steering angle
} WheelsInfo;

WheelsInfo WheelsInfo_create(float lf_speed,
                             float rf_speed,
                             float lb_speed,
                             float rb_speed,
                             float steering);

WheelsInfo WheelsInfo_default(void);

void WheelsInfo_toString(const WheelsInfo* wheels,
                         char* buffer,
                         int bufferSize);

#ifdef __cplusplus
}
#endif

#endif // WHEELSINFO_H
