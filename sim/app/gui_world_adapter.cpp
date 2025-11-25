#include "gui_world_adapter.hpp"

namespace fsai::sim::app {

GuiWorldSnapshot GuiWorldAdapter::snapshot() const {
  GuiWorldSnapshot snap{};
  snap.start_cones = world_.start_cones();
  snap.left_cones = world_.left_cones();
  snap.right_cones = world_.right_cones();
  snap.checkpoints = world_.checkpoint_positions();
  snap.lookahead = world_.lookahead_indices();
  snap.vehicle_transform = world_.vehicle_transform();
  snap.vehicle_state = world_.vehicle_state();
  snap.mission_runtime = world_.mission_runtime();
  snap.controller_path_edges = world_.controller_path_edges();
  snap.detections = world_.debug_detections();
  snap.pending_reset_reason = world_.pending_reset_reason();
  snap.lap_time_seconds = world_.lap_time_seconds();
  snap.total_distance_meters = world_.total_distance_meters();
  snap.time_step_seconds = world_.time_step_seconds();
  snap.lap_count = world_.lap_count();
  return snap;
}

}  // namespace fsai::sim::app

