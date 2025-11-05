#define _USE_MATH_DEFINES

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>

#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "VehicleState.hpp"
#include "Vector.h"
#include "Transform.h"
#include "types.h"

#include <typeinfo>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <cstring>

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;
using AllEdgeIterator=Triangulation::All_edges_iterator;
using FiniteEdgeIterator=Triangulation::Finite_edges_iterator;
using VertexHandle=Triangulation::Vertex_handle;

// Prints CGAL edges to standard out
void printEdges(Triangulation& T);

// Draws CGAL edges to CGAL Basic viewer
void drawEdges(Triangulation& T, CGAL::Graphics_scene& scene, CGAL::Color color = CGAL::IO::Color(15, 15, 15));

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
void getVisibleTrackTriangulation(
  Triangulation& T,
  Point carFront,
  TrackResult fullTrack,
  double sensorRange = 35.0,
  double sensorFOV = 2 * M_PI / 3
);


Triangulation getVisibleTrackTriangulation(
  Point carFront,
  double carYaw,
  std::vector<Vector3> leftConePositions,
  std::vector<Vector3> rightConePositions,
  double sensorRange = 35.0,
  double sensorFOV = 2 * M_PI / 3
);

std::vector<std::pair<Vector2, Vector2>> getVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Vector3>& leftConePositions,
  const std::vector<Vector3>& rightConePositions
);

void drawVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Vector3>& leftConePositions,
  const std::vector<Vector3>& rightConePositions
);