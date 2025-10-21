#ifndef WHEELSINFO_H
#define WHEELSINFO_H

#ifdef __cplusplus
extern "C" {
#endif

// Structure representing the wheel speeds and steering angle.
typedef struct {
    float lf_speed;   // Left-Front wheel speed
    float rf_speed;   // Right-Front wheel speed
    float lb_speed;   // Left-Rear wheel speed
    float rb_speed;   // Right-Rear wheel speed
    float steering;   // Steering angle
} WheelsInfo;

// "Constructor": Creates a WheelsInfo instance with specified values.
WheelsInfo WheelsInfo_create(float lf_speed, float rf_speed, float lb_speed, float rb_speed, float steering);

// Creates a default WheelsInfo with all fields set to zero.
WheelsInfo WheelsInfo_default();

// Writes a string representation of the WheelsInfo into the provided buffer.
// The caller must supply a buffer of at least bufferSize bytes.
void WheelsInfo_toString(const WheelsInfo* wheels, char* buffer, int bufferSize);

#ifdef __cplusplus
}
#endif

#endif // WHEELSINFO_H
