#define _USE_MATH_DEFINES

#include "centerline.prot.hpp"
#include "centerline.hpp"
#include "types.h"

#include <algorithm>

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;
using AllEdgeIterator=Triangulation::All_edges_iterator;
using FiniteEdgeIterator=Triangulation::Finite_edges_iterator;
using VertexHandle=Triangulation::Vertex_handle;

using namespace std;


// Debug method, leaving this here because iterating triangulation edges is weird in CGAL
void printEdges(Triangulation& T) {
  for (auto it = T.finite_edges_begin(); it != T.finite_edges_end(); ++it) {
        auto face_handle = it->first;
        int edge_index = it->second;

        // Get the two vertices of the edge
        auto v1 = face_handle->vertex((edge_index + 1) % 3);
        auto v2 = face_handle->vertex((edge_index + 2) % 3);

        // Get coordinates
        auto p1 = v1->point();
        auto p2 = v2->point();

        std::cout << "Edge: (" << p1.x() << "," << p1.y() << ") -> "
                  << "(" << p2.x() << "," << p2.y() << ")" << '\n';
    }
    std::cout << std::endl;
  }

std::pair<std::vector<PathNode>, std::vector<std::vector<int>>> generateGraph(
    Triangulation& T, CGAL::Graphics_scene& scene, Point carFront)
{
    std::vector<PathNode> nodes;
    // map each triangulation vertex to all node-ids that touch it
    std::map<Point, std::vector<int>> vertex_to_node_ids;

    // create a node for every triangulation edge we consider “drivable”
    for(auto it = T.finite_edges_begin(); it != T.finite_edges_end(); ++it)
    {
        auto seg = T.segment(*it);
        Point p1 = seg.source();
        Point p2 = seg.target();

        PathNode node;
        node.id = static_cast<int>(nodes.size());

        // midpoint
        node.midpoint.x = static_cast<float>((p1.x() + p2.x()) / 2.0);
        node.midpoint.y = static_cast<float>((p1.y() + p2.y()) / 2.0);

        // endpoints as “detections” (set what you actually need)
        FsaiConeDet d1{}; d1.x = static_cast<float>(p1.x()); d1.y = static_cast<float>(p1.y());
        d1.side = FSAI_CONE_UNKNOWN;
        FsaiConeDet d2{}; d2.x = static_cast<float>(p2.x()); d2.y = static_cast<float>(p2.y());
        d2.side = FSAI_CONE_UNKNOWN;

        node.first  = d1;
        node.second = d2;

        vertex_to_node_ids[p1].push_back(node.id);
        vertex_to_node_ids[p2].push_back(node.id);

        nodes.push_back(node);
    }

    // build index-based adjacency
    std::vector<std::vector<int>> adj(nodes.size());
    for (auto& kv : vertex_to_node_ids)
    {
        auto& ids = kv.second;
        for (size_t i = 0; i < ids.size(); ++i)
            for (size_t j = i + 1; j < ids.size(); ++j)
            {
                adj[ids[i]].push_back(ids[j]);
                adj[ids[j]].push_back(ids[i]);
            }
    }

    // pick start by nearest midpoint to carFront; draw its segment in red
    int start_id = 0;
    double best = std::numeric_limits<double>::infinity();
    for (auto& n : nodes)
    {
        const double dx = n.midpoint.x - static_cast<float>(carFront.x());
        const double dy = n.midpoint.y - static_cast<float>(carFront.y());
        const double d2 = dx*dx + dy*dy;
        if (d2 < best) { best = d2; start_id = n.id; }
    }

    scene.add_segment(Point(nodes[start_id].first.x,  nodes[start_id].first.y),
                      Point(nodes[start_id].second.x, nodes[start_id].second.y),
                      CGAL::IO::Color(250,15,15));

    return {nodes, adj};
}


void drawEdges(
    const std::vector<std::vector<int>>& adjacency,
    const std::vector<PathNode>& nodes,
    CGAL::Graphics_scene& scene,
    CGAL::Color color)
{
    // Avoid double-drawing: only draw i->j when i < j
    for (std::size_t i = 0; i < adjacency.size(); ++i) {
        for (int j : adjacency[i]) {
            if (static_cast<std::size_t>(j) <= i) continue;
            const auto& a = nodes[i].midpoint;
            const auto& b = nodes[j].midpoint;
            scene.add_segment(Point(a.x, a.y), Point(b.x, b.y), color);
        }
    }
}

void drawEdges(std::map<PathNode, std::set<PathNode>>& adjacency, CGAL::Graphics_scene& scene, CGAL::Color color) {
  for (auto & [node, adj_nodes]: adjacency) {
    for (auto adj_node: adj_nodes) {
      scene.add_segment(
        Point(node.midpoint.x,    node.midpoint.y),
        Point(adj_node.midpoint.x, adj_node.midpoint.y),
        color
      );
    }
  }
}

void drawEdges(Triangulation& T, CGAL::Graphics_scene& scene, CGAL::Color color) {
  for (auto it = T.finite_edges_begin(); it != T.finite_edges_end(); ++it) {
    auto segment = T.segment(*it);
    scene.add_segment(segment.source(), segment.target(), color);
  }
}

double getInitialTrackYaw(TrackResult track) {
  return std::atan2(
    track.checkpoints[1].position.z - track.checkpoints[0].position.z,
    track.checkpoints[1].position.x - track.checkpoints[0].position.x
  );
}

// Gets the angle between two direction vectors using Point as the data structure
double getAngle(Point a, Point b) {
  double dot = a.x()*b.x() + a.y()*b.y();
  // std::cout << "angle: " << std::acos(dot / (hypot(a.x(), a.y()) * hypot(b.x(), b.y())));
  return std::acos(dot / (hypot(a.x(), a.y()) * hypot(b.x(), b.y())));
}


// Gets the angle between two direction vectors using Vector2 as the data structure
double getAngle(Vector2 a, Vector2 b) {
  double dot = a.x*b.x + a.y*b.y;
  return std::acos(dot / (hypot(a.x, a.y) * hypot(b.x, b.y)));
}

// Returns the front of the car as a Point and modifies the triangulation in place
Point generateVehicleTriangulation(
  Triangulation& T,
  const TrackResult& track,
  double carLength,
  double carWidth
) {
      double carYaw = getInitialTrackYaw(track);

      double carX = track.checkpoints[0].position.x;
      double carZ = track.checkpoints[0].position.z;

      float frontX = carX + (carLength / 2.0) * std::cos(carYaw);
      float frontZ = carZ + (carLength / 2.0) * std::sin(carYaw);
      Point carFront = Point(frontX, frontZ);
      T.insert(carFront);

      float rearLeftX = carX - (carLength / 2.0) * std::cos(carYaw) - (carWidth / 2.0) * std::sin(carYaw);
      float rearLeftZ = carZ - (carLength / 2.0) * std::sin(carYaw) + (carWidth / 2.0) * std::cos(carYaw);
      T.insert(Point(rearLeftX, rearLeftZ));

      float rearRightX = carX - (carLength / 2.0) * std::cos(carYaw) + (carWidth / 2.0) * std::sin(carYaw);
      float rearRightZ = carZ - (carLength / 2.0) * std::sin(carYaw) - (carWidth / 2.0) * std::cos(carYaw);
      T.insert(Point(rearRightX, rearRightZ));

      return carFront;
};

// Returns the front of the car as a Vector2
Point getCarFront(
  VehicleState carState,
  double carLength,
  double carWidth
) {
      double carYaw = carState.yaw;

      double carX = carState.position.x();
      double carZ = carState.position.z();

      float frontX = carX + (carLength / 2.0) * std::cos(carYaw);
      float frontZ = carZ + (carLength / 2.0) * std::sin(carYaw);
      Point carFront = Point(frontX, frontZ);
      return carFront;
};


void getVisibleTrackTriangulation(
  Triangulation& T,
  Point carFront,
  TrackResult fullTrack,
  double sensorRange,
  double sensorFOV
) {
    double carYaw = getInitialTrackYaw(fullTrack);
    Point carVector = Point(std::cos(carYaw), std::sin(carYaw));

    // It can be assumed that leftCones.size() == rightCones.size() == checkpoints.size()
    auto addCones = [carFront, carVector, sensorRange, sensorFOV, &T](std::vector<Transform> cones) {
      for (int i = 0; i < cones.size(); i++) {
        Point delta = Point(cones[i].position.x - carFront.x(), cones[i].position.z - carFront.y());
        Point cone = Point(cones[i].position.x, cones[i].position.z);
        if (hypot(delta.x(), delta.y()) > sensorRange) continue;
        if (getAngle(carVector, delta) > sensorFOV/2) continue;
        T.insert(cone);
      }
    };

    addCones(fullTrack.startCones);
    addCones(fullTrack.leftCones);
    addCones(fullTrack.rightCones);
}

Triangulation getVisibleTrackTriangulation(
  Point carFront,
  double carYaw,
  std::vector<Cone> leftConePositions,
  std::vector<Cone> rightConePositions,
  double sensorRange,
  double sensorFOV
) {
    Triangulation visibleTrack;
    Point carVector = Point(std::cos(carYaw), std::sin(carYaw));

    auto addCones = [carFront, carVector, sensorRange, sensorFOV, &visibleTrack](std::vector<Cone> cones) {
      for (Cone cone3d: cones) {
        Point delta = Point(cone3d.position.x - carFront.x(), cone3d.position.z - carFront.y());
        Point cone = Point(cone3d.position.x, cone3d.position.z);
        if (std::hypot(delta.x(), delta.y()) > sensorRange) continue;
        if (getAngle(carVector, delta) > sensorFOV/2) continue;
        visibleTrack.insert(cone);
      }
    };

    addCones(leftConePositions);
    addCones(rightConePositions);

    return visibleTrack;
}


void drawVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
) {
    CGAL::Graphics_scene scene;
    Point carFront = Point(carState.position.x(), carState.position.y());
    auto triangulation = getVisibleTrackTriangulation(carFront, carState.yaw, leftConePositions, rightConePositions);
    CGAL::draw(triangulation);
}

std::vector<std::pair<Vector2, Vector2>> getVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
) {

    Point carFront = Point(carState.position.x(), carState.position.y());
    auto triangulation = getVisibleTrackTriangulation(carFront, carState.yaw , leftConePositions, rightConePositions);

    std::vector<std::pair<Vector2, Vector2>> edges {};
    for (auto it = triangulation.finite_edges_begin(); it != triangulation.finite_edges_end(); ++it) {
        auto face_handle = it->first;
        int edge_index = it->second;

        // Get the two vertices of the edge
        auto v1 = face_handle->vertex((edge_index + 1) % 3);
        auto v2 = face_handle->vertex((edge_index + 2) % 3);

        // Get coordinates
        auto p1 = v1->point();
        auto p2 = v2->point();

        edges.push_back({
          {
            static_cast<float>(p1.x()),
            static_cast<float>(p1.y())
          }, {
            static_cast<float>(p2.x()),
            static_cast<float>(p2.y())
          }});
    }

    return edges;
}


// --- Beam-search enumerator over midpoint graph ---
// We enumerate simple paths (no repeated nodes) up to maxLen using a beam search.
// For each partial path with >= 2 nodes we compute calculateCost(path)
// and keep the best one seen. Start node is chosen as the node whose
// midpoint is closest to carFront.
// centerline.prot.cpp
std::vector<PathNode> bfsLowestCost(
    const std::vector<std::vector<int>>& adj,
    const std::vector<PathNode>& nodes,
    const Point& carFront,
    std::size_t maxLen,
    std::size_t beamWidth
)
{
    if (nodes.empty()) return {};

    // pick start (same logic as above)
    int start = 0; double best = std::numeric_limits<double>::infinity();
    for (auto& n : nodes) {
        double d = std::hypot(n.midpoint.x - carFront.x(), n.midpoint.y - carFront.y());
        if (d < best) { best = d; start = n.id; }
    }

    const std::size_t effectiveBeamWidth = std::max<std::size_t>(1, beamWidth);

    struct BeamState {
        std::vector<int> path;
        float score;
    };

    std::vector<int> bestPathIdx = {start};
    float bestCost = std::numeric_limits<float>::infinity();

    auto toNodes = [&](const std::vector<int>& pathIdx) {
        std::vector<PathNode> path;
        path.reserve(pathIdx.size());
        for (int id : pathIdx) {
            path.push_back(nodes[id]);
        }
        return path;
    };

    auto evaluatePath = [&](const std::vector<int>& pathIdx) {
        if (pathIdx.size() < 2) {
            return std::numeric_limits<float>::infinity();
        }
        auto path = toNodes(pathIdx);
        float cost = calculateCost(path);
        if (cost < bestCost) {
            bestCost = cost;
            bestPathIdx = pathIdx;
        }
        // Encourage longer paths when costs tie.
        return cost - static_cast<float>(pathIdx.size()) * 1e-3f;
    };

    std::vector<BeamState> frontier;
    frontier.push_back({{start}, std::numeric_limits<float>::infinity()});

    while (!frontier.empty()) {
        std::vector<BeamState> expanded;
        expanded.reserve(frontier.size() * 2);

        for (auto& state : frontier) {
            // Update best path with current state if valid
            evaluatePath(state.path);

            if (state.path.size() >= maxLen) {
                continue;
            }

            int last = state.path.back();
            for (int next : adj[last]) {
                if (std::find(state.path.begin(), state.path.end(), next) != state.path.end()) {
                    continue;
                }
                auto nextPath = state.path;
                nextPath.push_back(next);
                float score = evaluatePath(nextPath);
                expanded.push_back({std::move(nextPath), score});
            }
        }

        if (expanded.empty()) {
            break;
        }

        std::sort(expanded.begin(), expanded.end(), [](const BeamState& a, const BeamState& b) {
            return a.score < b.score;
        });

        if (expanded.size() > effectiveBeamWidth) {
            expanded.resize(effectiveBeamWidth);
        }

        frontier = std::move(expanded);
    }

    if (bestPathIdx.size() < 2) {
        return {nodes[start]};
    }

    auto bestPath = toNodes(bestPathIdx);
    return bestPath;
}



// Draws segments between consecutive midpoints in a path.
void drawPathMidpoints(
    const std::vector<PathNode>& path,
    CGAL::Graphics_scene& scene,
    CGAL::Color color)
{
    if(path.size() < 2)
    {
      return;
    }

    for(std::size_t i = 1; i < path.size(); i++)
    {
        scene.add_segment(
            Point(path[i-1].midpoint.x, path[i-1].midpoint.y),
            Point(path[i].midpoint.x,   path[i].midpoint.y),
            color
        );
    }
}
