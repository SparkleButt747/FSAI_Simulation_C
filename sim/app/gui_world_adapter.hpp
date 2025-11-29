#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "Vector.h"
#include "Transform.h"
#include "VehicleState.hpp"
#include "sim/MissionRuntimeState.hpp"
#include "sim/WorldRuntime.hpp"
#include "sim/architecture/IWorldView.hpp"

namespace fsai::sim::app {

struct GuiWorldSnapshot {
  std::vector<Vector3> start_cones;
  std::vector<Vector3> left_cones;
  std::vector<Vector3> right_cones;
  std::vector<Vector3> orange_cones;
  std::vector<Vector3> checkpoints;
  LookaheadIndices lookahead{};
  Transform vehicle_transform{};
  VehicleState vehicle_state{};
  fsai::sim::MissionRuntimeState mission_runtime{};
  std::vector<std::pair<Vector2, Vector2>> controller_path_edges;
  std::vector<FsaiConeDet> detections;
  std::optional<fsai::sim::WorldRuntime::ResetReason> pending_reset_reason{};
  double lap_time_seconds{0.0};
  double total_distance_meters{0.0};
  double time_step_seconds{0.0};
  int lap_count{0};
};

class GuiWorldAdapter {
 public:
  explicit GuiWorldAdapter(const fsai::world::IWorldView& world) : world_(world) {}

  GuiWorldSnapshot snapshot() const;

 private:
  const fsai::world::IWorldView& world_;
};

}  // namespace fsai::sim::app

