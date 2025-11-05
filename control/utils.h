#ifndef UTILS_H
#define UTILS_H


#include <vector>
#include "globals.h"


// Distance helpers
float dist(float ax, float ay, float bx, float by);
float dist(const Vector2& a, const Vector2& b);
float dist(const FsaiConeDet& a, const FsaiConeDet& b);


// Vector/angle helpers
float vecLen(float x, float y);
float angleBetween(const Vector2& a, const Vector2& b, const Vector2& c);


// Statistics
float stdev(const std::vector<float>& v);


#endif // UTILS_H