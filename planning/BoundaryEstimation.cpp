#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>

#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "Transform.h"

#include <typeinfo>
#include <chrono>
#include <iostream>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Triangulation;
typedef Triangulation::Point          Point;
typedef Triangulation::All_edges_iterator edge_iterator;
typedef Triangulation::All_faces_iterator face_iterator;

int main()
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    PathConfig pathConfig;
    int nPoints = pathConfig.resolution;
    PathGenerator pathGen(pathConfig);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(pathConfig, path);

    Triangulation T;

    auto t1 = high_resolution_clock::now();
    for (int i = 0; i < track.leftCones.size(); i++) {
      T.insert(Point(track.leftCones[i].position.x, track.leftCones[i].position.z));
    }
    for (int i = 0; i < track.rightCones.size(); i++) {
      T.insert(Point(track.rightCones[i].position.x, track.rightCones[i].position.z));
    }
    auto t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t2 - t1;

    std::cout << "Triangulation insert: " << ms_double.count() << "ms\n";

    for (edge_iterator it = T.all_edges_begin(); it != T.all_edges_end(); ++it) {
        auto face_handle = it->first;
        int edge_index = it->second;

        // Get the two vertices of the edge
        auto v1 = face_handle->vertex((edge_index + 1) % 3);
        auto v2 = face_handle->vertex((edge_index + 2) % 3);

        // Get coordinates
        auto p1 = v1->point();
        auto p2 = v2->point();

        std::cout << "Edge: (" << p1.x() << "," << p1.y() << ") -> "
                  << "(" << p2.x() << "," << p2.y() << ")" << std::endl;
    }

  CGAL::draw(T);
  return 0;
}
