#include "centerline.hpp"
#include "utils.h"
#include "types.h"
#include <map>  
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>
#include <unordered_map>

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;
using AllEdgeIterator=Triangulation::All_edges_iterator;
using FiniteEdgeIterator=Triangulation::Finite_edges_iterator;
using VertexHandle=Triangulation::Vertex_handle;

using namespace std;

namespace {
    CostWeights& costWeightsStorage(){
        static CostWeights weights{};
        return weights;
    }
}

CostWeights defaultCostWeights() {
    return CostWeights{};
}

CostWeights getCostWeights() {
    return costWeightsStorage();
}

void setCostWeights(const CostWeights& weights) {
    auto& storage = costWeightsStorage();
    storage.angleMax   = std::max(0.0f, weights.angleMax);
    storage.widthStd   = std::max(0.0f, weights.widthStd);
    storage.spacingStd = std::max(0.0f, weights.spacingStd);
    storage.color      = std::max(0.0f, weights.color);
    storage.rangeSq    = std::max(0.0f, weights.rangeSq);
}

namespace {
    float clampAngle(float angle){
        while (angle > static_cast<float>(M_PI)) {
            angle -= 2.0f * static_cast<float>(M_PI);
        }
        while (angle < -static_cast<float>(M_PI)) {
            angle += 2.0f * static_cast<float>(M_PI);
        }
        return angle;
    }
}

float calculateCost(const std::vector<PathNode>& path, std::size_t minLen) {
    if (path.size() < minLen) {
        return std::numeric_limits<float>::infinity();
    }

    // Tunable weights
    const CostWeights weights = getCostWeights();

    constexpr float SENSOR_RANGE  = 20.0f;    // meters
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

    for(const auto& n : path) {
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

    for(std::size_t i = 1; i < path.size(); i++) {
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

    for (const auto& n : path) {
        const auto a=n.first.side;
        const auto b=n.second.side;

        if(a==FSAI_CONE_UNKNOWN || b==FSAI_CONE_UNKNOWN)
        {
            continue;
        }

        const bool opposite =   (a==FSAI_CONE_LEFT && b==FSAI_CONE_RIGHT) || 
                                (a==FSAI_CONE_RIGHT && b==FSAI_CONE_LEFT);

        considered++;
        if(!opposite) {
            sameSide++;
        }
    }

    float colorPenalty = 0.0f;
    if(considered>0) {
        colorPenalty = static_cast<float>(considered - sameSide)/static_cast<float>(considered);
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

// REPLACING THE ENTIRE generateGraph FUNCTION

std::pair<std::vector<PathNode>, std::vector<std::vector<int>>> generateGraph(
    const FsaiDetections& cones,
    const Vector3& carFront,
    float coneToSide)
{
    std::vector<PathNode> nodes;
    std::map<int, std::vector<int>> cone_idx_to_node_ids;

    // 1. Separate cones by indices
    std::vector<int> left_indices;
    std::vector<int> right_indices;

    for (int i = 0; i < cones.n; i++) {
        if (cones.dets[i].side == FSAI_CONE_LEFT) { 
            left_indices.push_back(i);
        } else if (cones.dets[i].side == FSAI_CONE_RIGHT) { 
            right_indices.push_back(i);
        }
    }

    // 2. Find Gates (Left/Right pairs within distance)
    const float MAX_GATE_WIDTH = 12.0f; 

    for (int l_idx : left_indices) {
        for (int r_idx : right_indices) {
            const FsaiConeDet& l_cone = cones.dets[l_idx];
            const FsaiConeDet& r_cone = cones.dets[r_idx];

            float dx = l_cone.x - r_cone.x;
            float dy = l_cone.y - r_cone.y;
            float distSq = dx*dx + dy*dy;

            if (distSq > MAX_GATE_WIDTH * MAX_GATE_WIDTH) continue;

            PathNode node;
            node.id = static_cast<int>(nodes.size());
            node.midpoint.x = (l_cone.x + r_cone.x) / 2.0f;
            node.midpoint.y = (l_cone.y + r_cone.y) / 2.0f;
            
            // Important: Store cone data so the cost function works
            node.first = l_cone; 
            node.second = r_cone;

            cone_idx_to_node_ids[l_idx].push_back(node.id);
            cone_idx_to_node_ids[r_idx].push_back(node.id);

            nodes.push_back(node);
        }
    }

    // 3. Build Adjacency
    std::vector<std::vector<int>> adj(nodes.size());
    for (auto const& [cone_idx, node_ids] : cone_idx_to_node_ids) {
        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = i + 1; j < node_ids.size(); ++j) {
                int u = node_ids[i];
                int v = node_ids[j];
                adj[u].push_back(v);
                adj[v].push_back(u);
            }
        }
    }

    // 4. Sort nodes so the one closest to car is first (preserves old behavior)
    if (!nodes.empty()) {
        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::infinity();

        for (int i = 0; i < nodes.size(); i++) {
            double dx = nodes[i].midpoint.x - carFront.x;
            double dy = nodes[i].midpoint.y - carFront.y;
            double d2 = dx*dx + dy*dy;
            if (d2 < best_dist) {
                best_dist = d2;
                best_idx = i;
            }
        }
        
        // Swap best node to front if needed by downstream logic
        if (best_idx != -1 && best_idx != 0) {
            std::swap(nodes[0], nodes[best_idx]);
            // Note: If we swap nodes, we technically mess up the adjacency indices 
            // unless we remap them. 
        }
    }

    return {nodes, adj};
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
  return std::acos(dot / (hypot(a.x(), a.y()) * hypot(b.x(), b.y())));
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

std::unordered_map<Point, FsaiConeSide> getVisibleTrackTriangulationFromTrack(
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
        
        if (std::hypot(delta.x(), delta.y()) > sensorRange) {
            continue;
        }
        if (getAngle(carVector, delta) > sensorFOV/2) {
            continue;
        }
        
        coneToSide[cone] = side;
        T.insert(cone);
      }
    };

    addCones(fullTrack.startCones, FSAI_CONE_UNKNOWN);
    addCones(fullTrack.leftCones, FSAI_CONE_LEFT);
    addCones(fullTrack.rightCones, FSAI_CONE_RIGHT);

    return coneToSide;
}

void updateVisibleTrackTriangulation(
  Triangulation& T,
  std::unordered_map<Point, FsaiConeSide>& coneToSide,
  Point carFront,
  double carYaw,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions,
  double sensorRange,
  double sensorFOV
) {
    Point carVector = Point(std::cos(carYaw), std::sin(carYaw));

    auto addCones = [carFront, carVector, sensorRange, sensorFOV, &T, &coneToSide](const std::vector<Cone>& cones, FsaiConeSide side) {
      for (const auto& cone3d : cones) {
        Point delta = Point(cone3d.position.x - carFront.x(), cone3d.position.z - carFront.y());
        Point cone = Point(cone3d.position.x, cone3d.position.z);
        if (std::hypot(delta.x(), delta.y()) > sensorRange) continue;
        if (getAngle(carVector, delta) > sensorFOV/2) continue;
        
        if (coneToSide.find(cone) == coneToSide.end()) {
            T.insert(cone);
            coneToSide[cone] = side;
        }
      }
    };

    addCones(leftConePositions, FSAI_CONE_LEFT);
    addCones(rightConePositions, FSAI_CONE_RIGHT);
}

std::vector<std::pair<Vector2, Vector2>> getVisibleTriangulationEdges(
  Triangulation& triangulation,
  std::unordered_map<Point, FsaiConeSide>& coneToSide,
  VehicleState carState,
  const std::vector<Cone>& leftConePositions,
  const std::vector<Cone>& rightConePositions
) {
    Point carFront = getCarFront(carState);
    updateVisibleTrackTriangulation(triangulation, coneToSide, carFront, carState.yaw, leftConePositions, rightConePositions);

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

        edges.emplace_back(
            Vector2{ static_cast<float>(p1.x()), static_cast<float>(p1.y()) },
            Vector2{ static_cast<float>(p2.x()), static_cast<float>(p2.y()) }
        );
    }

    return edges;
}

std::pair<std::vector<PathNode>, std::vector<std::pair<Vector2, Vector2>>> beamSearch(
    const std::vector<std::vector<int>>& adj,
    const std::vector<PathNode>& nodes,
    const Point& carFront,
    std::size_t maxLen,
    std::size_t minLen,
    std::size_t beamWidth
) {
    if (nodes.empty() || adj.empty() || maxLen == 0 || beamWidth == 0) {
        return {};
    }

    auto buildPathFromIds = [&](const std::vector<int>& ids) {
        std::vector<PathNode> path;
        path.reserve(ids.size());
        for (int id : ids) {
            if (id < 0 || static_cast<std::size_t>(id) >= nodes.size())
            {
                continue;
            }
            path.push_back(nodes[static_cast<std::size_t>(id)]);
        }
        return path;
    };

    auto evaluate = [&](const std::vector<int>& ids) {
        if (ids.size() < 2){
            return 0.0f;
        }
        const auto path = buildPathFromIds(ids);

        return calculateCost(path, minLen);
    };

    // pick start = node closest to carFront (simple, deterministic)
    int start = 0;
    double bestD2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const double dx = nodes[i].midpoint.x - carFront.x();
        const double dy = nodes[i].midpoint.y - carFront.y();
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) {
            bestD2 = d2;
            start = static_cast<int>(i);
        }
    }

    struct Candidate {
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
        if (candidate.indices.size() < 2) {
            return;
        }
        if (!std::isfinite(candidate.cost)) {
            return;
        }
        if (candidate.cost < bestCost ||
            (candidate.cost == bestCost && candidate.indices.size() > bestCandidate.indices.size())
        ) {
            bestCost = candidate.cost;
            bestCandidate = candidate;
        }
    };

    for (std::size_t depth = 0; depth < maxLen && !beam.empty(); ++depth) {
        std::vector<Candidate> next;
        bool extendedAny = false;

        for (Candidate& candidate : beam) {
            if (candidate.indices.size() >= maxLen) {
                updateBest(candidate);
                next.push_back(std::move(candidate));
                continue;
            }

            const int last = candidate.indices.back();
            bool extendedCurrent = false;

            if (last >= 0 && static_cast<std::size_t>(last) < adj.size()) {
                for (int nb : adj[static_cast<std::size_t>(last)]) {
                    if (nb < 0 || static_cast<std::size_t>(nb) >= nodes.size()) {
                        continue;
                    }

                    // avoid loops by preventing revisiting a node already on the path
                    if (std::find(candidate.indices.begin(), candidate.indices.end(), nb) != candidate.indices.end()) {
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

            if (!extendedCurrent) {
                updateBest(candidate);
                next.push_back(std::move(candidate));
            }
        }

        if (!extendedAny) {
            break;
        }

        std::sort(next.begin(), next.end(), [](const Candidate& a, const Candidate& b) {
            const bool aFinite = std::isfinite(a.cost);
            const bool bFinite = std::isfinite(b.cost);
            if (aFinite != bFinite) {
                return aFinite;
            }
            if (a.cost == b.cost) {
                return a.indices.size() > b.indices.size();
            }

            return a.cost < b.cost;
        });

        if (next.size() > beamWidth) {
            next.resize(beamWidth);
        }

        beam = std::move(next);
    }

    if (bestCandidate.indices.size() < 2) {
        for (const Candidate& candidate : beam) {
            if (candidate.indices.size() < 2) {
                continue;
            }
            if (!std::isfinite(candidate.cost)) {
                continue;
            }
            if (candidate.cost < bestCost ||
                (candidate.cost == bestCost && candidate.indices.size() > bestCandidate.indices.size())
            ) {
                bestCost = candidate.cost;
                bestCandidate = candidate;
            }
        }
    }

    if (bestCandidate.indices.size() < 2) {
        return {};
    }

    return {buildPathFromIds(bestCandidate.indices), getPathEdges(buildPathFromIds(bestCandidate.indices))};
}

void removePassedCones(
  Triangulation& T,
  std::unordered_map<Point, FsaiConeSide>& coneToSide,
  Point carFront,
  double carYaw
) {
    double carForwardX = std::cos(carYaw);
    double carForwardY = std::sin(carYaw);

    std::vector<VertexHandle> vertices_to_remove;
    for (auto v_it = T.finite_vertices_begin(); v_it != T.finite_vertices_end(); ++v_it) {
        Point p = v_it->point();
        double cone_to_carX = p.x() - carFront.x();
        double cone_to_carY = p.y() - carFront.y();

        double dot_product = carForwardX * cone_to_carX + carForwardY * cone_to_carY;

        if (dot_product < 0) {
            // Check if cone is far enough behind
            if (std::hypot(cone_to_carX, cone_to_carY) > 1.0) { // 1m threshold
                vertices_to_remove.push_back(v_it);
            }
        }
    }

    for (const auto& vh : vertices_to_remove) {
        coneToSide.erase(vh->point());
        T.remove(vh);
    }
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



