#include "PathGenerator.hpp"
#include "TrackGenerator.hpp"
#include "Transform.h"

#include <fstream>
#include <sstream>

int main() {
    srand(3); // seed goes here, different per test case
    PathConfig pathConfig;
    PathGenerator pathGen(pathConfig);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(pathConfig, path);

}