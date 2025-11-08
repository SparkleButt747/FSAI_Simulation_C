#ifndef FSAI_CENTERLINE_HPP
#define FSAI_CENTERLINE_HPP

#include "types.h"
#include "Vector.h"
#include <vector>
#include <tuple>
#include "types.h"

/** This represents a node on the path with the code detections that correspond
 * to the two points of the edges being included for computing the cost of a
 * path. There is also a special case where the PathNode is the starting node
 * which is the car's position. In this case, left and right are null.
 */
class PathNode {
  public:
    int id = -1;
    Vector2 midpoint;
    FsaiConeDet first;
    FsaiConeDet second;
    std::vector<PathNode*> children;

    bool operator==(const PathNode& other) const
    {
        return midpoint.x == other.midpoint.x && midpoint.y == other.midpoint.y && first.x == other.first.x && first.y == other.first.y;
    }

    bool operator<(const PathNode& other) const
    {
        return midpoint.x < other.midpoint.x;
    }
};

/**
 * Takes an unordered list of cone detections and returns a centerline
 * represented as the list of the checkpoints (or midpoints) between the left
 * and right cones which make up the track. See below for the full process
 *
 * 1. Calculate the Delaunay triangulation using the CGAL Library (done in prot)
 * 2. Find the tree of the midpoints of the edges which make up the triangulation
 * 3. Explore the tree of possible paths from step 2 in a breadth first order
 * 4. Prune the tree as you go based on the cost of the path
 *
 * see section 4.3.1 here: https://arxiv.org/pdf/1905.05150
 */
std::vector<Vector2> getCenterline(
    std::vector<FsaiConeDet> coneDetections,
    FsaiVehicleState vehicleState
);


/**
 * Cost is calculated as a combination of these factors:
 * - Change in angle: Sharp are unlikely since there will be multiple cone
 *   detections for real tight turns on the track
 * - Standard deviation of the track width: The track width is regulated by the
 *   rules and should not change much
 * - Standard deviation of the distance between left as well as the right cones:
 *   Cones corresponding to the track are normally roughly space
 *   equal
 * - Color probability: Based on the rules, the left of a PathNode should be
 *   blue and the right should be yellow. The cost should be zero if there is
 *   missing information
 * - Squared difference between path length and sensor range:
 *   The cost penalizes too short and too long paths
 *   and gives an emphasize to paths that are the same length
 *   as the sensor range which is around 10 m
 *
 * see table 3 here: https://arxiv.org/pdf/1905.05150
 */
float calculateCost(std::vector<PathNode> path);


// Ideas
// Work out left and right cones relative to the car with cross product

#endif // FSAI_CENTERLINE_HPP