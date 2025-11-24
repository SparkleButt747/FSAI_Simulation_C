#pragma once

#include <string>

#include "World.hpp"

namespace fsai::sim {

class WorldConfigLoader {
public:
    static WorldConfig FromFile(const std::string& path, const MissionDefinition& mission);
};

}  // namespace fsai::sim

