#include "centerline.hpp"
#include "utils.h"
#include "types.h"


#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

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

float calculateCost(const std::vector<PathNode>& path)
{
    if (path.size() < 2) {
        return std::numeric_limits<float>::infinity();
    }


    // Tunable weights
    const CostWeights weights = getCostWeights();

    constexpr float SENSOR_RANGE  = 10.0f;    // meters
    constexpr float NOMINAL_TRACK_WIDTH = 3.5f;  // see FS Driverless rules / AMZ Driverless


    // 1) Curvature and smoothness (AMZ Driverless Section 4.3.1 / Table 3)
    float maxTurn = 0.0f;
    float turnAccumSq = 0.0f;
    std::size_t turnSamples = 0;

    std::vector<float> headings;
    headings.reserve(path.size() - 1);

    for (std::size_t i = 1; i < path.size(); ++i) {
        const auto& prev = path[i - 1].midpoint;
        const auto& next = path[i].midpoint;
        const float dx = next.x - prev.x;
        const float dy = next.y - prev.y;
        if (dx == 0.0f && dy == 0.0f) {
            headings.push_back(headings.empty() ? 0.0f : headings.back());
            continue;
        }
        float heading = std::atan2(dy, dx);
        if (!headings.empty()) {
            heading = clampAngle(headings.back() + clampAngle(heading - headings.back()));
        }
        headings.push_back(heading);
    }

    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        float ang = angleBetween(path[i - 1].midpoint, path[i].midpoint, path[i + 1].midpoint);
        float turn = static_cast<float>(M_PI) - ang;
        turn = std::max(0.0f, turn);
        maxTurn = std::max(maxTurn, turn);
        turnAccumSq += turn * turn;
        ++turnSamples;
    }

    float headingStd = 0.0f;
    if (headings.size() >= 2) {
        headingStd = stdev(headings);
    }

    float curvatureScore = 0.0f;
    if (turnSamples > 0) {
        const float rmsTurn = std::sqrt(turnAccumSq / static_cast<float>(turnSamples));
        curvatureScore = 0.6f * maxTurn + 0.4f * rmsTurn;
    }
    curvatureScore += 0.25f * headingStd;


    // 2) Standard deviation of track width
    std::vector<float> widths;
    widths.reserve(path.size());

    for(const auto& n : path)
    {
        widths.push_back(dist(n.first.x, n.first.y, n.second.x, n.second.y));
    }

    float widthStd = stdev(widths);
    float widthMean = 0.0f;
    if (!widths.empty()) {
        widthMean = std::accumulate(widths.begin(), widths.end(), 0.0f) / static_cast<float>(widths.size());
    }
    const float widthMeanDeviation = std::abs(widthMean - NOMINAL_TRACK_WIDTH);
    const float widthScore = 0.7f * widthStd + 0.3f * widthMeanDeviation;


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
    float leftMean = 0.0f;
    float rightMean = 0.0f;
    if (!leftSpacing.empty()) {
        leftMean = std::accumulate(leftSpacing.begin(), leftSpacing.end(), 0.0f) /
                   static_cast<float>(leftSpacing.size());
    }
    if (!rightSpacing.empty()) {
        rightMean = std::accumulate(rightSpacing.begin(), rightSpacing.end(), 0.0f) /
                    static_cast<float>(rightSpacing.size());
    }
    const float spacingAsymmetry = std::abs(leftMean - rightMean);
    const float spacingScore = spacingStd + 0.2f * spacingAsymmetry;


    // 4) Color penalty, 0 if any color info missing, else 1 per mismatch
    int sameSide=0;
    int considered=0;

    for (const auto& n : path)
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

    if (path.size() < 4)
    {
        // Encourage the beam search to keep extending short candidates.
        const float minUsefulLength = 0.5f * SENSOR_RANGE;
        const float shortfall = std::max(0.0f, minUsefulLength - pathLen);
        rangeCost += shortfall * shortfall;
    }

    // Weighted sum
    const float cost =
        weights.angleMax   * curvatureScore +
        weights.widthStd   * widthScore +
        weights.spacingStd * spacingScore +
        weights.color      * colorPenalty +
        weights.rangeSq    * rangeCost;

    return cost;
}