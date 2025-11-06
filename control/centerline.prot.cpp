#define _USE_MATH_DEFINES

#include "centerline.prot.hpp"

using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation=CGAL::Delaunay_triangulation_2<K>;
using Point=Triangulation::Point;
using AllEdgeIterator=Triangulation::All_edges_iterator;
using FiniteEdgeIterator=Triangulation::Finite_edges_iterator;
using VertexHandle=Triangulation::Vertex_handle;

class PathNode {
  public:
    Point midpoint;
    Point first;
    Point second;
    std::vector<PathNode*> children;

    bool operator==(const PathNode& other) const
    {
        return midpoint == other.midpoint && first == other.first;
    }

    bool operator<(const PathNode& other) const
    {
        return midpoint < other.midpoint;
    }
};


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

std::pair<std::vector<PathNode>, std::map<PathNode, std::set<PathNode>>> generateGraph(
  Triangulation& T,
  CGAL::Graphics_scene& scene,
  Point carFront
) {
  std::vector<PathNode> nodes {};
  std::map<Point, std::vector<PathNode>> vertex_to_nodes {};
  scene.add_text(carFront, "car");
  for (auto it = T.finite_edges_begin(); it != T.finite_edges_end(); ++it) {
        auto face_handle = it->first;
        int edge_index = it->second;

        // Get the two vertices of the edge
        auto v1 = face_handle->vertex((edge_index + 1) % 3);
        auto v2 = face_handle->vertex((edge_index + 2) % 3);

        // Get coordinates
        auto p1 = v1->point();
        auto p2 = v2->point();

        scene.add_point(p1, CGAL::IO::Color(200, 200, 10));
        scene.add_point(p2, CGAL::IO::Color(200, 200, 10));

        PathNode node = {{(p1.x() + p2.x())/2, (p1.y() + p2.y())/2}, p1, p2, {}};
        nodes.push_back(node);
        vertex_to_nodes[p1].push_back(node);
        vertex_to_nodes[p2].push_back(node);
    }

    std::map<PathNode, std::set<PathNode>> adjacency {};
    for (auto & [v, incident]: vertex_to_nodes) {
      for (int i = 0; i < incident.size(); i++) {
        for (int j = i+1; j < incident.size(); j++) {
          auto node1 = incident[i];
          auto node2 = incident[j];
          adjacency[node1].insert(node2);
          adjacency[node2].insert(node1);
        }
      }
    }

    PathNode startNode = nodes[0];
    double dist = INFINITY;
    for (auto node: nodes) {
      double currentDist = std::hypot(node.midpoint.x() - carFront.x(), node.midpoint.y() - carFront.y());
      if (currentDist < dist) {
        dist = currentDist;
        startNode = node;
      }
    }

    scene.add_segment(startNode.first, startNode.second, CGAL::IO::Color(250, 15, 15));
    return {nodes, adjacency};
  }

void drawEdges(std::map<PathNode, std::set<PathNode>>& adjacency, CGAL::Graphics_scene& scene, CGAL::Color color) {
  for (auto & [node, adj_nodes]: adjacency) {
    for (auto adj_node: adj_nodes) {
      scene.add_segment(node.midpoint, adj_node.midpoint, color);
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
    printEdges(triangulation);

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