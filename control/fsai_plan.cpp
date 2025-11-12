#include "centerline.prot.cpp"

int main(int argc, char* argv[])
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    // Optional seed random number generator.
    if (argc > 1) {
      srand(atoi(argv[1]));
      std::cout << "Seed from cmd args: " << argv[1] << '\n';
    } else {
      unsigned seed = (unsigned) time(NULL);
      std::cout << "Seed: " << seed << '\n';
      srand(seed);
    }


    PathConfig pathConfig = PathConfig(5);
    int nPoints = pathConfig.resolution;
    PathGenerator pathGen(pathConfig);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(pathConfig, path);

    Triangulation T;
    Triangulation CarT;

    Point carFront = generateVehicleTriangulation(CarT, track);

    auto t1 = high_resolution_clock::now();
    for (int i = 0; i < track.leftCones.size(); i++) {
      T.insert(Point(track.leftCones[i].position.x, track.leftCones[i].position.z));
    }
    for (int i = 0; i < track.rightCones.size(); i++) {
      T.insert(Point(track.rightCones[i].position.x, track.rightCones[i].position.z));
    }
    auto t2 = high_resolution_clock::now();

    duration<double, std::milli> ms_double = t2 - t1;

    std::cout << "Triangulation insert: " << ms_double.count() << "ms\n";

    Triangulation visibleT;
    getVisibleTrackTriangulation(visibleT, carFront, track);
    CGAL::Graphics_scene scene;
    std::map<PathNode, std::set<PathNode>> adjacency = generateGraph(visibleT, scene, carFront).second;
    drawEdges(adjacency, scene, CGAL::IO::Color(15, 250, 15));
    // Optional viewing with car position and visible cones
    drawEdges(visibleT, scene, CGAL::IO::Color(15, 15, 250));
    drawEdges(T, scene);
    CGAL::add_to_graphics_scene(CarT, scene);
    CGAL::draw_graphics_scene(scene);

    return EXIT_SUCCESS;
}
