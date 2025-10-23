#include "sim/integration/csv_logger.hpp"

#include <utility>

namespace fsai::sim::integration {
namespace {
void closeFile(std::FILE*& file) {
  if (file) {
    std::fclose(file);
    file = nullptr;
  }
}
}  // namespace

CsvLogger::CsvLogger(const std::string& state_path,
                     const std::string& control_path) {
  state_log_ = std::fopen(state_path.c_str(), "w");
  control_log_ = std::fopen(control_path.c_str(), "w");

  if (state_log_) {
    std::fprintf(state_log_, "time,x,y,z,yaw,v_x,v_y,v_z\n");
  }
  if (control_log_) {
    std::fprintf(control_log_, "time,throttle,steering\n");
  }

  if (!state_log_ || !control_log_) {
    closeFile(state_log_);
    closeFile(control_log_);
  }
}

CsvLogger::~CsvLogger() {
  closeFile(state_log_);
  closeFile(control_log_);
}

void CsvLogger::logState(double time_seconds, const VehicleState& state) const {
  if (!state_log_) {
    return;
  }

  std::fprintf(state_log_,
               "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               time_seconds,
               state.position.x(),
               state.position.y(),
               state.position.z(),
               state.yaw,
               state.velocity.x(),
               state.velocity.y(),
               state.velocity.z());
}

void CsvLogger::logControl(double time_seconds,
                           float throttle,
                           float steering) const {
  if (!control_log_) {
    return;
  }

  std::fprintf(control_log_, "%.6f,%.6f,%.6f\n",
               time_seconds,
               static_cast<double>(throttle),
               static_cast<double>(steering));
}

}  // namespace fsai::sim::integration
