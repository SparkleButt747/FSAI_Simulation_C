#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "MissionRuntimeState.hpp"
#include "Transform.h"
#include "VehicleState.hpp"
#include "Vector.h"
#include "WheelsInfo.h"
#include "common/types.h"

namespace fsai::world {

/**
 * Read-only view of the simulated environment and vehicle ground truth.
 * Implementations should avoid exposing mutable state so that control/vision
 * consumers can only observe through this boundary.
 */
class IWorldView {
 public:
  virtual ~IWorldView() = default;

  // --- Vehicle ground truth ---
  virtual const VehicleState& vehicle_state() const = 0;
  virtual const Transform& vehicle_transform() const = 0;
  virtual const WheelsInfo& wheels_info() const = 0;

  // --- Track geometry ---
  virtual const std::vector<Vector3>& checkpoint_positions() const = 0;
  virtual const std::vector<Vector3>& start_cones() const = 0;
  virtual const std::vector<Vector3>& left_cones() const = 0;
  virtual const std::vector<Vector3>& right_cones() const = 0;
  virtual const LookaheadIndices& lookahead_indices() const = 0;
  virtual const std::vector<std::pair<Vector2, Vector2>>& best_path_edges() const = 0;

  // --- Mission/runtime bookkeeping ---
  virtual const fsai::sim::MissionRuntimeState& mission_runtime() const = 0;
  virtual double lap_time_seconds() const = 0;
  virtual double total_distance_meters() const = 0;
  virtual double time_step_seconds() const = 0;
  virtual int lap_count() const = 0;

  // --- Sensor/vision ground truth ---
  virtual const std::vector<FsaiConeDet>& ground_truth_detections() const = 0;

  // --- Lifecycle helpers for consumers holding stale state ---
  virtual bool vehicle_reset_pending() const = 0;
  virtual void acknowledge_vehicle_reset(const Transform& applied_transform) = 0;
};

}  // namespace fsai::world

