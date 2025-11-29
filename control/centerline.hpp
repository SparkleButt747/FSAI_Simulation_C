#ifndef FSAI_CENTERLINE_HPP
#define FSAI_CENTERLINE_HPP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "VehicleState.hpp"
#include "Vector.h"
#include "Transform.h"
#include "types.h"
#include "sim/world/TrackTypes.hpp"


#include <tuple>
#include <typeinfo>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <cstring>
#include <unordered_map>
#include <functional>

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;

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

    bool operator==(const PathNode& other) const {
        return midpoint.x == other.midpoint.x && midpoint.y == other.midpoint.y && first.x == other.first.x && first.y == other.first.y;
    }

    bool operator<(const PathNode& other) const {
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
float calculateCost(const std::vector<PathNode>& path, std::size_t minLen);

float calculateCost_Acceleration(const std::vector<PathNode>& path, std::size_t minLen);

struct CostWeights {
    float angleMax = 5.3;
    float widthStd = 2.1f;
    float spacingStd = 10.0f;
    float color = 10.0f;
    float rangeSq = 0.15f;
};

CostWeights defaultCostWeights();
CostWeights getCostWeights();
void setCostWeights(const CostWeights& weights);

// Build a PathNode graph from a visible triangulation
std::pair<std::vector<PathNode>, std::vector<std::vector<int>>> generateGraph(
    Triangulation& T,
    Point carFront,
    std::unordered_map<Point, FsaiConeSide> coneToSide
);

double getInitialTrackYaw(TrackResult track);

// Gets the angle between two direction vectors using Point as the data structure
double getAngle(Point a, Point b);

// Returns the front of the car as a Point and modifies the triangulation in
// place, representing the car visually as a triangle.
Point generateVehicleTriangulation(
  Triangulation& T,
  const TrackResult& track,
  double carLength = 4.0,
  double carWidth = 1.5
);

Point getCarFront(
  VehicleState carState,
  double carLength = 4.0,
  double carWidth = 1.5
);

// Modifies the triangulation of the cones visible from the car position assuming
// a circular sector veiwing area
std::unordered_map<Point, FsaiConeSide> getVisibleTrackTriangulationFromTrack(
  Triangulation& T,
  Point carFront,
  TrackResult fullTrack,
  double sensorRange = 20.0,
  double sensorFOV = 2 * M_PI / 3
);

void updateVisibleTrackTriangulation(
  Triangulation& T,
  std::unordered_map<Point, FsaiConeSide>& coneToSide,
  Point carFront,
  double carYaw,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions,
  double sensorRange = 20.0,
  double sensorFOV = 2 * M_PI / 3
);

void removePassedCones(
  Triangulation& T,
  std::unordered_map<Point, FsaiConeSide>& coneToSide,
  Point carFront,
  double carYaw
);

std::pair<Triangulation, std::unordered_map<Point, FsaiConeSide>> getVisibleTrackTriangulationFromCones(
  Point carFront,
  double carYaw,
  std::vector<Cone> leftConePositions,
  std::vector<Cone> rightConePositions,
  double sensorRange = 20.0,
  double sensorFOV = 2 * M_PI / 3
);

std::pair<Triangulation,
std::unordered_map<Point, FsaiConeSide>> getVisibleTrackTriangulationFromCones(
  Point carFront,
  double carYaw,
  std::vector<Cone> leftConePositions,
  std::vector<Cone> rightConePositions,
  std::vector<Cone> orangeConePositions,
  double sensorRange = 20.0,
  double sensorFOV = 2 * M_PI / 3
);

std::pair<Triangulation, std::vector<std::pair<Vector2, Vector2>>> getVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
);

std::pair<Triangulation, std::vector<std::pair<Vector2, Vector2>>> getVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions,
  const std::vector<Cone>& orangeConePositions
);

// Returns a simple path (sequence of PathNode) with the lowest cost according to calculateCost.
// Explores candidate paths up to maxLen nodes using a beam search whose width can be tuned.
std::pair<std::vector<PathNode>, std::vector<std::pair<Vector2, Vector2>>> beamSearch(
    const std::vector<std::vector<int>>& adj,
    const std::vector<PathNode>& nodes,
    const Point& carFront,
    std::size_t maxLen,
    std::size_t minLen,
    std::size_t beamWidth
);

std::pair<std::vector<PathNode>, std::vector<std::pair<Vector2, Vector2>>> beamSearch(
    const std::vector<std::vector<int>>& adj,
    const std::vector<PathNode>& nodes,
    const Point& carFront,
    std::size_t maxLen,
    std::size_t minLen,
    std::size_t beamWidth,
    const std::function<float(const std::vector<PathNode>&, std::size_t)>& costFunc
);

std::vector<std::pair<Vector2, Vector2>> getPathEdges(const std::vector<PathNode>& path);

Vector3* pathNodesToCheckpoints(std::vector<PathNode> path);

// Ideas
// Work out left and right cones relative to the car with cross product

#endif // FSAI_CENTERLINE_HPP