#define _USE_MATH_DEFINES

//#include "centerline.prot.hpp"
#include "centerline.hpp"
#include "types.h"

#include <algorithm>
#include <unordered_map>
#include <cmath>

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;
using AllEdgeIterator=Triangulation::All_edges_iterator;
using FiniteEdgeIterator=Triangulation::Finite_edges_iterator;
using VertexHandle=Triangulation::Vertex_handle;

using namespace std;


/*
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
  }*/

std::pair<std::vector<PathNode>, std::vector<std::vector<int>>> generateGraph(
    Triangulation& T, Point carFront, std::unordered_map<Point, FsaiConeSide> coneToSide)
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
        auto it1 = coneToSide.find(p1);
        d1.side = (it1!=coneToSide.end()) ? it1->second : FSAI_CONE_UNKNOWN;
        FsaiConeDet d2{}; d2.x = static_cast<float>(p2.x()); d2.y = static_cast<float>(p2.y());
        auto it2 = coneToSide.find(p2);
        d2.side = (it2!=coneToSide.end()) ? it2->second : FSAI_CONE_UNKNOWN;

        if (d1.side == d2.side) continue;

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

    // scene.add_segment(Point(nodes[start_id].first.x,  nodes[start_id].first.y),
    //                   Point(nodes[start_id].second.x, nodes[start_id].second.y),
    //                   CGAL::IO::Color(250,15,15));

    return {nodes, adj};
}


// void drawEdges(
//     const std::vector<std::vector<int>>& adjacency,
//     const std::vector<PathNode>& nodes,
//     CGAL::Graphics_scene& scene,
//     CGAL::Color color)
// {
//     // Avoid double-drawing: only draw i->j when i < j
//     for (std::size_t i = 0; i < adjacency.size(); ++i) {
//         for (int j : adjacency[i]) {
//             if (static_cast<std::size_t>(j) <= i) continue;
//             const auto& a = nodes[i].midpoint;
//             const auto& b = nodes[j].midpoint;
//             scene.add_segment(Point(a.x, a.y), Point(b.x, b.y), color);
//         }
//     }
// }

// void drawEdges(std::map<PathNode, std::set<PathNode>>& adjacency, CGAL::Graphics_scene& scene, CGAL::Color color) {
//   for (auto & [node, adj_nodes]: adjacency) {
//     for (auto adj_node: adj_nodes) {
//       scene.add_segment(
//         Point(node.midpoint.x,    node.midpoint.y),
//         Point(adj_node.midpoint.x, adj_node.midpoint.y),
//         color
//       );
//     }
//   }
// }

// void drawEdges(Triangulation& T, CGAL::Graphics_scene& scene, CGAL::Color color) {
//   for (auto it = T.finite_edges_begin(); it != T.finite_edges_end(); ++it) {
//     auto segment = T.segment(*it);
//     scene.add_segment(segment.source(), segment.target(), color);
//   }
// }

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

/* Only the getAngle with point params is used...
// Gets the angle between two direction vectors using Vector2 as the data structure
double getAngle(Vector2 a, Vector2 b) {
  double dot = a.x*b.x + a.y*b.y;
  return std::acos(dot / (hypot(a.x, a.y) * hypot(b.x, b.y)));
}*/

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


std::unordered_map<Point, FsaiConeSide> getVisibleTrackTriangulation(
  Triangulation& T,
  Point carFront,
  TrackResult fullTrack,
  double sensorRange,
  double sensorFOV
) {
    double carYaw = getInitialTrackYaw(fullTrack);
    Point carVector = Point(std::cos(carYaw), std::sin(carYaw));
    std::unordered_map<Point, FsaiConeSide> coneToSide;
    // It can be assumed that leftCones.size() == rightCones.size() == checkpoints.size()
    auto addCones = [carFront, carVector, sensorRange, sensorFOV, &T, &coneToSide](std::vector<Transform> cones, FsaiConeSide side) {
      for (int i = 0; i < cones.size(); i++) {
        Point delta = Point(cones[i].position.x - carFront.x(), cones[i].position.z - carFront.y());
        Point cone = Point(cones[i].position.x, cones[i].position.z);
        if (std::hypot(delta.x(), delta.y()) > sensorRange) continue;
        if (getAngle(carVector, delta) > sensorFOV/2) continue;
        coneToSide[cone] = side;
        T.insert(cone);
      }
    };

    addCones(fullTrack.startCones, FSAI_CONE_UNKNOWN);
    addCones(fullTrack.leftCones, FSAI_CONE_LEFT);
    addCones(fullTrack.rightCones, FSAI_CONE_RIGHT);
    return coneToSide;
}

std::pair<Triangulation, std::unordered_map<Point, FsaiConeSide>> getVisibleTrackTriangulation(
  Point carFront,
  double carYaw,
  std::vector<Cone> leftConePositions,
  std::vector<Cone> rightConePositions,
  double sensorRange,
  double sensorFOV
) {
    Triangulation visibleTrack;
    Point carVector = Point(std::cos(carYaw), std::sin(carYaw));
    std::unordered_map<Point, FsaiConeSide> coneToSide;

    auto addCones = [carFront, carVector, sensorRange, sensorFOV, &visibleTrack, &coneToSide](std::vector<Cone> cones, FsaiConeSide side) {
      for (Cone cone3d: cones) {
        Point delta = Point(cone3d.position.x - carFront.x(), cone3d.position.z - carFront.y());
        Point cone = Point(cone3d.position.x, cone3d.position.z);
        if (std::hypot(delta.x(), delta.y()) > sensorRange) continue;
        if (getAngle(carVector, delta) > sensorFOV/2) continue;
        coneToSide[cone] = side;
        visibleTrack.insert(cone);
      }
    };

    addCones(leftConePositions, FSAI_CONE_LEFT);
    addCones(rightConePositions, FSAI_CONE_RIGHT);

    return {visibleTrack, coneToSide};
}


// void drawVisibleTriangulationEdges(
//   VehicleState carState,
//   const std::vector<Cone>& leftConePositions,
//   const std::vector<Cone>& rightConePositions
// ) {
//     CGAL::Graphics_scene scene;
//     Point carFront = Point(carState.position.x(), carState.position.y());
//     auto triangulation = getVisibleTrackTriangulation(carFront, carState.yaw, leftConePositions, rightConePositions).first;
//     CGAL::draw(triangulation);
// }

std::pair<Triangulation, std::vector<std::pair<Vector2, Vector2>>> getVisibleTriangulationEdges(
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
) {

    Point carFront = Point(carState.position.x(), carState.position.y());
    //auto triangulation = getVisibleTrackTriangulation(carFront, carState.yaw , leftConePositions, rightConePositions);
    auto triangulation_pair = getVisibleTrackTriangulation(carFront, carState.yaw, leftConePositions, rightConePositions);
    auto triangulation = triangulation_pair.first;


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

        /*
        edges.push_back({
          {
            static_cast<float>(p1.x()),
            static_cast<float>(p1.y())
          }, {
            static_cast<float>(p2.x()),
            static_cast<float>(p2.y())
          }});*/
          edges.emplace_back(
          Vector2{ static_cast<float>(p1.x()), static_cast<float>(p1.y()) },
          Vector2{ static_cast<float>(p2.x()), static_cast<float>(p2.y()) }
          );
    }

    return {triangulation, edges};
}

std::pair<std::vector<PathNode>, std::vector<std::pair<Vector2, Vector2>>> beamSearch(
    const std::vector<std::vector<int>>& adj,
    const std::vector<PathNode>& nodes,
    const Point& carFront,
    std::size_t maxLen,
    std::size_t minLen,
    std::size_t beamWidth
)
{
    if (nodes.empty() || adj.empty() || maxLen == 0 || beamWidth == 0)
    {
        return {};
    }

    auto buildPathFromIds = [&](const std::vector<int>& ids) {
        std::vector<PathNode> path;
        path.reserve(ids.size());
        for (int id : ids)
        {
            if (id < 0 || static_cast<std::size_t>(id) >= nodes.size())
            {
                continue;
            }
            path.push_back(nodes[static_cast<std::size_t>(id)]);
        }
        return path;
    };

    auto evaluate = [&](const std::vector<int>& ids) {
        if (ids.size() < 2)
        {
            return 0.0f;
        }
        const auto path = buildPathFromIds(ids);
        return calculateCost(path, minLen);
    };

    // pick start = node closest to carFront (simple, deterministic)
    int start = 0;
    double bestD2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        const double dx = nodes[i].midpoint.x - carFront.x();
        const double dy = nodes[i].midpoint.y - carFront.y();
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2)
        {
            bestD2 = d2;
            start = static_cast<int>(i);
        }
    }

    struct Candidate
    {
        std::vector<int> indices;
        float cost;

        Candidate() = default;
        Candidate(std::vector<int> idx, float c) : indices(std::move(idx)), cost(c) {}
    };

    std::vector<Candidate> beam;
    beam.emplace_back(std::vector<int>{start}, 0.0f);

    Candidate bestCandidate = beam.front();
    float bestCost = std::numeric_limits<float>::infinity();

    auto updateBest = [&](const Candidate& candidate) {
        if (candidate.indices.size() < 2)
        {
            return;
        }
        if (!std::isfinite(candidate.cost))
        {
            return;
        }
        if (candidate.cost < bestCost ||
            (candidate.cost == bestCost && candidate.indices.size() > bestCandidate.indices.size()))
        {
            bestCost = candidate.cost;
            bestCandidate = candidate;
        }
    };

    for (std::size_t depth = 0; depth < maxLen && !beam.empty(); ++depth)
    {
        std::vector<Candidate> next;
        bool extendedAny = false;

        for (Candidate& candidate : beam)
        {
            if (candidate.indices.size() >= maxLen)
            {
                updateBest(candidate);
                next.push_back(std::move(candidate));
                continue;
            }

            const int last = candidate.indices.back();
            bool extendedCurrent = false;

            if (last >= 0 && static_cast<std::size_t>(last) < adj.size())
            {
                for (int nb : adj[static_cast<std::size_t>(last)])
                {
                    if (nb < 0 || static_cast<std::size_t>(nb) >= nodes.size())
                    {
                        continue;
                    }

                    // avoid loops by preventing revisiting a node already on the path
                    if (std::find(candidate.indices.begin(), candidate.indices.end(), nb) != candidate.indices.end())
                    {
                        continue;
                    }

                    std::vector<int> indices = candidate.indices;
                    indices.push_back(nb);
                    const float score = evaluate(indices);

                    Candidate expanded{std::move(indices), score};
                    updateBest(expanded);
                    next.push_back(std::move(expanded));
                    extendedCurrent = true;
                    extendedAny = true;
                }
            }

            if (!extendedCurrent)
            {
                updateBest(candidate);
                next.push_back(std::move(candidate));
            }
        }

        if (!extendedAny)
        {
            break;
        }

        std::sort(next.begin(), next.end(), [](const Candidate& a, const Candidate& b) {
            const bool aFinite = std::isfinite(a.cost);
            const bool bFinite = std::isfinite(b.cost);
            if (aFinite != bFinite)
            {
                return aFinite;
            }
            if (a.cost == b.cost)
            {
                return a.indices.size() > b.indices.size();
            }
            return a.cost < b.cost;
        });

        if (next.size() > beamWidth)
        {
            next.resize(beamWidth);
        }

        beam = std::move(next);
    }

    if (bestCandidate.indices.size() < 2)
    {
        for (const Candidate& candidate : beam)
        {
            if (candidate.indices.size() < 2)
            {
                continue;
            }
            if (!std::isfinite(candidate.cost))
            {
                continue;
            }
            if (candidate.cost < bestCost ||
                (candidate.cost == bestCost && candidate.indices.size() > bestCandidate.indices.size()))
            {
                bestCost = candidate.cost;
                bestCandidate = candidate;
            }
        }
    }

    if (bestCandidate.indices.size() < 2)
    {
        return {};
    }

    return {buildPathFromIds(bestCandidate.indices), getPathEdges(buildPathFromIds(bestCandidate.indices))};
}

std::vector<std::pair<Vector2, Vector2>> getPathEdges(const std::vector<PathNode>& path) {
    std::vector<std::pair<Vector2, Vector2>> edges;
    for (std::size_t i = 1; i < path.size(); i++) {
        edges.emplace_back(
            Vector2{path[i-1].midpoint.x, path[i-1].midpoint.y},
            Vector2{path[i].midpoint.x, path[i].midpoint.y}
        );
    }
    return edges;
}


// Draws segments between consecutive midpoints in a path.
// void drawPathMidpoints(
//     const std::vector<PathNode>& path,
//     CGAL::Graphics_scene& scene,
//     CGAL::Color color)
// {
//     if(path.size() < 2)
//     {
//       return;
//     }

//     for(std::size_t i = 1; i < path.size(); i++)
//     {
//         scene.add_segment(
//             Point(path[i-1].midpoint.x, path[i-1].midpoint.y),
//             Point(path[i].midpoint.x,   path[i].midpoint.y),
//             color
//         );
//     }
// }

Vector3* pathNodesToCheckpoints(std::vector<PathNode> path) {
    Vector3* checkpoints = new Vector3[path.size()];
    for (int i = 0; i < path.size(); i++) {
        checkpoints[i] = Vector3{
            path[i].midpoint.x,
            0,
            path[i].midpoint.y
        };
    }
    return checkpoints;
}
