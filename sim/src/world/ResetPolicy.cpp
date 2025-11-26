#include "ResetPolicy.hpp"

ResetPolicy::ResetPolicy(const WorldControlConfig& controlConfig)
    : controlConfig_(controlConfig) {}

ResetDecision ResetPolicy::Decide(fsai::sim::WorldRuntime::ResetReason reason,
                                  const fsai::sim::MissionDefinition& mission) const {
    ResetDecision decision{};

    if (!mission.allowRegeneration || !controlConfig_.regenerateTrack) {
        return decision;
    }

    switch (reason) {
        case fsai::sim::WorldRuntime::ResetReason::kTrackRegeneration:
        case fsai::sim::WorldRuntime::ResetReason::kConeCollision:
        case fsai::sim::WorldRuntime::ResetReason::kBoundaryCollision:
            decision.regenerateTrack = true;
            break;
    }

    return decision;
}

