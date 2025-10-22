#include "Vector.h"
#include <math.h>

Vector2 Vector2_Add(const Vector2 a, const Vector2 b) {
    Vector2 result = { a.x + b.x, a.y + b.y };
    return result;
}

Vector2 Vector2_Subtract(const Vector2 a, const Vector2 b) {
    Vector2 result = { a.x - b.x, a.y - b.y };
    return result;
}

Vector2 Vector2_Scale(const Vector2 v, float s) {
    Vector2 result = { v.x * s, v.y * s };
    return result;
}

float Vector2_Dot(const Vector2 a, const Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

float Vector2_Magnitude(const Vector2 v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

Vector2 Vector2_Normalize(const Vector2 v) {
    float mag = Vector2_Magnitude(v);
    if (mag == 0) return v;
    return Vector2_Scale(v, 1.0f / mag);
}

float Vector2_SignedAngle(const Vector2 from, const Vector2 to) {
    // Compute the angle between the two vectors using atan2.
    float angleFrom = atan2f(from.y, from.x);
    float angleTo   = atan2f(to.y, to.x);
    float angle = angleTo - angleFrom;
    // Convert radians to degrees.
    angle = angle * (180.0f / (float)M_PI);
    // Normalize to [-180, 180].
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

Vector3 Vector3_Add(const Vector3 a, const Vector3 b) {
    Vector3 result = { a.x + b.x, a.y + b.y, a.z + b.z };
    return result;
}

Vector3 Vector3_Subtract(const Vector3 a, const Vector3 b) {
    Vector3 result = { a.x - b.x, a.y - b.y, a.z - b.z };
    return result;
}

Vector3 Vector3_Scale(const Vector3 v, float s) {
    Vector3 result = { v.x * s, v.y * s, v.z * s };
    return result;
}

float Vector3_Dot(const Vector3 a, const Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float Vector3_Magnitude(const Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 Vector3_Normalize(const Vector3 v) {
    float mag = Vector3_Magnitude(v);
    if (mag == 0) return v;
    return Vector3_Scale(v, 1.0f / mag);
}
