#ifndef VECTOR_H
#define VECTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float x;
        float y;
    } Vector2;

        // LookaheadIndices structure.
    typedef struct
    {
        int steer;
        int speed;
        int max;

    } LookaheadIndices;

    typedef struct
    {
        float x;
        float y;
        float z;
    } Vector3;

    // Vector2 Operations
    Vector2 Vector2_Add(const Vector2 a, const Vector2 b);
    Vector2 Vector2_Subtract(const Vector2 a, const Vector2 b);
    Vector2 Vector2_Scale(const Vector2 v, float s);
    float Vector2_Dot(const Vector2 a, const Vector2 b);
    float Vector2_Magnitude(const Vector2 v);
    Vector2 Vector2_Normalize(const Vector2 v);
    // Returns the signed angle (in degrees) from vector 'from' to vector 'to'
    float Vector2_SignedAngle(const Vector2 from, const Vector2 to);

    // Vector3 Operations
    Vector3 Vector3_Add(const Vector3 a, const Vector3 b);
    Vector3 Vector3_Subtract(const Vector3 a, const Vector3 b);
    Vector3 Vector3_Scale(const Vector3 v, float s);
    float Vector3_Dot(const Vector3 a, const Vector3 b);
    float Vector3_Magnitude(const Vector3 v);
    Vector3 Vector3_Normalize(const Vector3 v);

#ifdef __cplusplus
}
#endif

#endif // VECTOR_H
