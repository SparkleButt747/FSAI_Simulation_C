#define _USE_MATH_DEFINES

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "VehicleState.hpp"
#include "Vector.h"
#include "World.hpp"
#include "Transform.h"
#include "types.h"
#include "centerline.hpp"

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

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;
using AllEdgeIterator=Triangulation::All_edges_iterator;
using FiniteEdgeIterator=Triangulation::Finite_edges_iterator;
using VertexHandle=Triangulation::Vertex_handle;

// Build a PathNode graph from a visible triangulation
std::pair<std::vector<PathNode>, std::vector<std::vector<int>>> generateGraph(
    Triangulation& T,
    Point carFront,
    std::unordered_map<Point, FsaiConeSide> coneToSide
);

// Convenience drawer for the PathNode adjacency
// void drawEdges(
//   std::map<PathNode, std::set<PathNode>>& adjacency,
//   CGAL::Graphics_scene& scene,
//   CGAL::Color color = CGAL::IO::Color(15, 15, 15)
// );

// Prints CGAL edges to standard out
//void printEdges(Triangulation& T);

// Draws CGAL edges to CGAL Basic viewer
// void drawEdges(Triangulation& T, CGAL::Graphics_scene& scene, CGAL::Color color = CGAL::IO::Color(15, 15, 15));

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


std::pair<Triangulation, std::unordered_map<Point, FsaiConeSide>> getVisibleTrackTriangulationFromCones(
  Point carFront,
  double carYaw,
  std::vector<Cone> leftConePositions,
  std::vector<Cone> rightConePositions,
  double sensorRange = 20.0,
  double sensorFOV = 2 * M_PI / 3
);

std::pair<Triangulation, std::vector<std::pair<Vector2, Vector2>>> getVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
);

/*
void drawVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
);*/

// NEW: draw edges for an adjacency list indexed by node id
// void drawEdges(
//   const std::vector<std::vector<int>>& adjacency,
//   const std::vector<PathNode>& nodes,
//   CGAL::Graphics_scene& scene,
//   CGAL::Color color = CGAL::IO::Color(15, 15, 15));


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

std::vector<std::pair<Vector2, Vector2>> getPathEdges(const std::vector<PathNode>& path);

// Draw a path (by connecting PathNode midpoints).
// void drawPathMidpoints(
//   const std::vector<PathNode>& path,
//   CGAL::Graphics_scene& scene,
//   CGAL::Color color = CGAL::IO::Color(255, 100, 30)
// );

Vector3* pathNodesToCheckpoints(std::vector<PathNode> path);