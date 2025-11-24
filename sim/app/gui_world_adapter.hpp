#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "Vector.h"
#include "Transform.h"
#include "VehicleState.hpp"
#include "MissionRuntimeState.hpp"
#include "architecture/IWorldView.hpp"
#include "architecture/WorldDebugPacket.hpp"

namespace fsai::sim::app {

struct GuiWorldSnapshot {
  std::vector<Vector3> start_cones;
  std::vector<Vector3> left_cones;
  std::vector<Vector3> right_cones;
  std::vector<Vector3> checkpoints;
  LookaheadIndices lookahead{};
  Transform vehicle_transform{};
  VehicleState vehicle_state{};
  fsai::sim::MissionRuntimeState mission_runtime{};
  double lap_time_seconds{0.0};
  double total_distance_meters{0.0};
  double time_step_seconds{0.0};
  int lap_count{0};
  std::optional<fsai::world::WorldDebugPacket> debug;
};

class GuiWorldAdapter {
 public:
  GuiWorldAdapter(const fsai::world::IWorldView& world,
                  std::optional<fsai::world::WorldDebugPacket> debug)
      : world_(world), debug_(std::move(debug)) {}

  GuiWorldSnapshot snapshot() const;

 private:
  const fsai::world::IWorldView& world_;
  std::optional<fsai::world::WorldDebugPacket> debug_{};
};

}  // namespace fsai::sim::app

