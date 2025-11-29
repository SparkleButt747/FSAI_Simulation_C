#pragma once

#include "sim/WorldConfig.hpp"
#include "sim/WorldRuntime.hpp"

struct ResetDecision {
    bool regenerateTrack{false};
};

class ResetPolicy {
public:
    explicit ResetPolicy(const WorldControlConfig& controlConfig);

    ResetDecision Decide(fsai::sim::WorldRuntime::ResetReason reason,
                         const fsai::sim::MissionDefinition& mission) const;

private:
    WorldControlConfig controlConfig_{};
};

