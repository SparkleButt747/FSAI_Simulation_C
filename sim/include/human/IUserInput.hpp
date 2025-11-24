#pragma once

#include <optional>

#include "mission/MissionDefinition.hpp"

namespace human {

class IUserInput {
 public:
  virtual ~IUserInput() = default;

  virtual void PublishEmergencyStop() = 0;
  virtual void PublishMissionOverride(
      const fsai::sim::MissionDescriptor& mission) = 0;

  virtual bool ConsumeEmergencyStop() = 0;
  virtual std::optional<fsai::sim::MissionDescriptor> ConsumeMissionOverride() = 0;
};

}  // namespace human

