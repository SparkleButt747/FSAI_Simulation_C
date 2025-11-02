#include "utils.h"


#include <algorithm>
#include <vector>
#include <cmath>
#include <numeric>


float dist(float ax, float ay, float bx, float by)
{
    const float dx = ax - bx;
    const float dy = ay - by;
    return std::sqrt(dx*dx + dy*dy);
}


float dist(const Vector2& a, const Vector2& b)
{
    return dist(a.x, a.y, b.x, b.y);
}


float dist(const FsaiConeDet& a, const FsaiConeDet& b)
{
    return dist(a.x, a.y, b.x, b.y);
}


float vecLen(float x, float y)
{
    return std::sqrt(x*x + y*y);
}


float angleBetween(const Vector2& prev, const Vector2& pivot, const Vector2& next)
{
    // Compute vectors from the pivot
    // u = pivot->prev, v = pivot->next
    const float ux = prev.x - pivot.x;
    const float uy = prev.y - pivot.y;
    const float vx = next.x - pivot.x;
    const float vy = next.y - pivot.y;

    // Lengths
    const float lenU = vecLen(ux, uy);
    const float lenV = vecLen(vx, vy);

    // Bad segment => define angle as 0
    if(lenU == 0.0f or lenV == 0.0f)
    {
        return 0.0f;
    }

    // Dot product and cosine of the angle
    const float dot = ux*vx + uy*vy;
    float cos = dot / (lenU * lenV);

    // Clamp to [-1, 1] to avoid errors
    // Theorethically could happen but I might be wrong
    cos = std::max(-1.0f, std::min(1.0f, cos));

    // Return angle in radians
    return std::acos(cos);
}



// I did not whrite standard deviation function so it might be wrong
float stdev(const std::vector<float>& v)
{
    if(v.size() < 2) return 0.0f;
    const float mean = std::accumulate(v.begin(), v.end(), 0.0f) / float(v.size());
    float acc = 0.0f;
    for(float x : v)
    {
        const float d = x - mean;
        acc += d*d;
    }
    return std::sqrt(acc / float(v.size() - 1));
}
