#pragma once

#include <cstdio>
#include <string>

#include "VehicleState.hpp"

namespace fsai::sim::integration {

class CsvLogger {
 public:
  CsvLogger(const std::string& state_path, const std::string& control_path);
  ~CsvLogger();

  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;

  bool valid() const { return state_log_ && control_log_; }

  void logState(double time_seconds, const VehicleState& state) const;
  void logControl(double time_seconds, float throttle, float steering) const;

 private:
  std::FILE* state_log_ = nullptr;
  std::FILE* control_log_ = nullptr;
};

}  // namespace fsai::sim::integration
