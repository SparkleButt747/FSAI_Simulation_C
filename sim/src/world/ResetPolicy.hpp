#pragma once

#include "sim/mission/MissionDefinition.hpp"

class ResetPolicy {
 public:
  struct Config {
    bool regenerateOnCollision{true};
    bool regenerateRandomTracksOnly{false};
  };

  ResetPolicy() = default;
  explicit ResetPolicy(const Config& config);

  bool ShouldRegenerate(const fsai::sim::MissionDefinition& mission) const;

 private:
  Config config_{};
};

