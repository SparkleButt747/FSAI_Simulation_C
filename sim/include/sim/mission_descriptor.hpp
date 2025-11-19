#pragma once

#include <string>

namespace fsai::sim {

enum class MissionType {
  kAcceleration,
  kSkidpad,
  kAutocross,
  kTrackdrive,
};

struct MissionDescriptor {
  MissionType type{MissionType::kAutocross};
  std::string name{"Autocross"};
  std::string short_name{"autocross"};
};

}  // namespace fsai::sim

