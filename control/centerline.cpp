#include "centerline.hpp"
#include "utils.h"
#include "types.h"


#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

namespace {

CostWeights& costWeightsStorage()
{
    static CostWeights weights{};
    return weights;
}

}

CostWeights defaultCostWeights()
{
    return CostWeights{};
}

CostWeights getCostWeights()
{
    return costWeightsStorage();
}

void setCostWeights(const CostWeights& weights)
{
    auto& storage = costWeightsStorage();
    storage.angleMax   = std::max(0.0f, weights.angleMax);
    storage.widthStd   = std::max(0.0f, weights.widthStd);
    storage.spacingStd = std::max(0.0f, weights.spacingStd);
    storage.color      = std::max(0.0f, weights.color);
    storage.rangeSq    = std::max(0.0f, weights.rangeSq);
}

namespace {

float clampAngle(float angle)
{
    while (angle > static_cast<float>(M_PI)) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

}  // namespace


float calculateCost(std::vector<PathNode> path)
{
    // discourage super-short paths
    if(path.size() < 3) return 1e3f;


    // Tunable weights
    const CostWeights weights = getCostWeights();

    constexpr float SENSOR_RANGE  = 10.0f;    // meters
    constexpr float NOMINAL_TRACK_WIDTH = 3.5f;


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
    int sameSide=0;
    int considered=0;

    for(const auto& n : path)
    {
        const auto a=n.first.side;
        const auto b=n.second.side;

        if(a==FSAI_CONE_UNKNOWN || b==FSAI_CONE_UNKNOWN)
        {
            continue;
        }

        const bool opposite =   (a==FSAI_CONE_LEFT && b==FSAI_CONE_RIGHT) || 
                                (a==FSAI_CONE_RIGHT && b==FSAI_CONE_LEFT);

        considered++;
        if(!opposite) 
        {
            sameSide++;
        }
    }

    float colorPenalty = 0.0f;
    if(considered>0) 
    {
        colorPenalty = static_cast<float>(sameSide)/static_cast<float>(considered);
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
        weights.angleMax   * maxAngleChange +
        weights.widthStd   * widthStd +
        weights.spacingStd * spacingStd +
        weights.color      * colorPenalty +
        weights.rangeSq    * rangeCost;

    return cost;
}