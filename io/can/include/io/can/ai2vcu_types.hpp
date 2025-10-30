#pragma once

#include <io/can/adsdv_dbc.hpp>

namespace fsai::io::can {

struct Ai2VcuCommandSet {
  fsai::sim::svcu::dbc::Ai2VcuStatus status;
  fsai::sim::svcu::dbc::Ai2VcuSteer steer;
  fsai::sim::svcu::dbc::Ai2VcuDrive front_drive;
  fsai::sim::svcu::dbc::Ai2VcuDrive rear_drive;
  fsai::sim::svcu::dbc::Ai2VcuBrake brake;
  float throttle_clamped{0.0f};
  float brake_clamped{0.0f};
};

}  // namespace fsai::io::can
