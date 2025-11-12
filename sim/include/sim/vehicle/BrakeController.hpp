#pragma once
#include "VehicleParam.hpp"

struct BrakeRequest;
struct BrakeStatus;

class BrakeController {
public:
  BrakeController();
  explicit BrakeController(const BrakeParam& p);

  void setParam(const BrakeParam& p);

  // Prepare ideal brake split from driver command.
  BrakeRequest prepare(double command01, double speed_mps) const;

  // Finalize given actual regen force available from powertrain.
  BrakeStatus finalize(const BrakeRequest& req, double regen_force_available) const;

private:
  void normalize_();

  BrakeParam P_{};
};
