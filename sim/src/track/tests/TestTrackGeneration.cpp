#include <iostream>
#include <fstream>
#include <ctime>
#include <sim/track/PathConfig.hpp>
#include <sim/track/PathGenerator.hpp>
#include <sim/track/TrackGenerator.hpp>

int main() {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    PathConfig config;
    int nPoints = config.resolution;
    PathGenerator pathGen(config);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(config, path);

    std::ofstream fp("left_cones.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.leftCones) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    fp.open("right_cones.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.rightCones) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    fp.open("checkpoints.csv");
    fp << "x,y,z,rotation\n";
    for (const auto& t : track.checkpoints) {
        fp << t.position.x << ',' << t.position.y << ',' << t.position.z << ',' << t.yaw << '\n';
    }
    fp.close();

    std::cout << "Data written to left_cones.csv, right_cones.csv, and checkpoints.csv" << std::endl;
    int ret = std::system("python ../PathLogic/plot_track.py");
    if (ret != 0) {
        std::cerr << "Graphing tool did not run successfully. Please run plot_track.py manually." << std::endl;
    }
    return 0;
}

