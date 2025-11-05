#include "centerline.hpp"
#include "utils.h"
#include "types.h"


#include <vector>
#include <cmath>


float calculateCost(std::vector<PathNode> path)
{
    if(path.size() < 2) return 0.0f;


    // Tunable weights
    constexpr float W_ANGLE_MAX   = 1.0f;     // radians
    constexpr float W_WIDTH_STD   = 2.0f;     // meters
    constexpr float W_SPACING_STD = 1.5f;     // meters
    constexpr float W_COLOR       = 3.0f;     // unitless
    constexpr float W_RANGE_SQ    = 0.2f;     // meters^2

    constexpr float SENSOR_RANGE  = 10.0f;    // meters


    // 1) Maximum angle change between consecutive midpoints
    float maxAngleChange = 0.0f;

    for(size_t i = 1; i + 1 < path.size(); i++)
    {
        float ang = angleBetween(path[i-1].midpoint, path[i].midpoint, path[i+1].midpoint);
        // We want CHANGE, i.e., deviation from straight (pi == 180°). Using turning angle:
        // Turning angle = pi - interior angle (0 for straight, larger for sharp turns).
        float turn = static_cast<float>(M_PI) - ang;
        if(turn > maxAngleChange)
        {
            maxAngleChange = turn;
        }
    }


    // 2) Standard deviation of track width
    std::vector<float> widths;
    widths.reserve(path.size());

    for(const auto& n : path)
    {
        widths.push_back(dist(n.first.x, n.first.y, n.second.x, n.second.y));
    }

    float widthStd = stdev(widths);


    // 3) Std dev of distances between consecutive left cones and consecutive right cones
    std::vector<float> leftSpacing, rightSpacing;
    leftSpacing.reserve(path.size());
    rightSpacing.reserve(path.size());

    for(std::size_t i = 1; i < path.size(); i++)
    {
        leftSpacing.push_back(dist(path[i-1].first, path[i].first));
        rightSpacing.push_back(dist(path[i-1].second, path[i].second));
    }

    float spacingStd = 0.5f * (stdev(leftSpacing) + stdev(rightSpacing)); // 0.5f bc leftSpacing + rightSpacing


    // 4) Color penalty, 0 if any color info missing, else 1 per mismatch
    bool anyUnknown = false;
    int mismatches = 0;

    for(const auto& n : path)
    {
        if(n.first.side == FSAI_CONE_UNKNOWN or n.second.side == FSAI_CONE_UNKNOWN)
        {
            anyUnknown = true;
            break;
        }
        
        if(n.first.side  != FSAI_CONE_LEFT)
        {
            mismatches++;
        }
        if(n.second.side != FSAI_CONE_RIGHT)
        {
            mismatches++;
        }
    }

    float colorPenalty;
    if(anyUnknown)
    {
        colorPenalty = 0.0f;
    }
    else
    {
        colorPenalty = static_cast<float>(mismatches);
    }


    // 5) Squared difference between path length and sensor range
    float pathLen = 0.0f;

    for(std::size_t i = 1; i < path.size(); i++)
    {
        pathLen += dist(path[i-1].midpoint, path[i].midpoint);
    }

    float rangeCost = (pathLen - SENSOR_RANGE);
    rangeCost *= rangeCost; // squared

    // Weighted sum
    const float cost =
        W_ANGLE_MAX   * maxAngleChange +
        W_WIDTH_STD   * widthStd +
        W_SPACING_STD * spacingStd +
        W_COLOR       * colorPenalty +
        W_RANGE_SQ    * rangeCost;
    
    return cost;
}