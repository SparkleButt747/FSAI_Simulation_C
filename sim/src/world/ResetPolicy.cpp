#include "ResetPolicy.hpp"

ResetPolicy::ResetPolicy(const Config& config) : config_(config) {}

bool ResetPolicy::ShouldRegenerate(const fsai::sim::MissionDefinition& mission) const {
    if (!config_.regenerateOnCollision) {
        return false;
    }
    if (!mission.allowRegeneration) {
        return false;
    }
    if (config_.regenerateRandomTracksOnly && mission.trackSource != fsai::sim::TrackSource::kRandom) {
        return false;
    }
    return true;
}

