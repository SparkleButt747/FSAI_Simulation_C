#include <algorithm>
#include <array>
#include <chrono>
#include <numbers>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <atomic>
#include <thread>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>
#include <limits>

#include <SDL.h>
#include <yaml-cpp/yaml.h>

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"

#include "Graphics.h"
#include "budget.h"
#include "fsai_clock.h"
#include "csv_logger.hpp"
#include "logging.hpp"
#include "provider_registry.hpp"
#include "stereo_display.hpp"
#include "sim_stereo_source.hpp"
#include "types.h"
#include "World.hpp"
#include "adsdv_dbc.hpp"
#include "link.hpp"
#include "can_link.hpp"
#include "io/can/can_transport.hpp"
#include "ai2vcu_adapter.hpp"
#include "can_iface.hpp"
#include "runtime_telemetry.hpp"
#include "svcu_runner.hpp"

namespace {
constexpr double kDefaultDt = 0.01;
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr int kReportIntervalFrames = 120;
constexpr float kRenderScale = 5.0f;
constexpr uint16_t kDefaultCommandPort = fsai::sim::svcu::kDefaultCommandPort;
constexpr uint16_t kDefaultTelemetryPort = fsai::sim::svcu::kDefaultTelemetryPort;
constexpr double kCommandStaleSeconds = 0.1;
constexpr double kAckLagWarningSeconds = 0.25;
constexpr double kBaseLatitudeDeg = 37.4275;
constexpr double kBaseLongitudeDeg = -122.1697;
constexpr double kMetersPerDegreeLat = 111111.0;
constexpr const char* kDefaultSensorConfig = "../configs/sim/sensors.yaml";

template <size_t Capacity>
class RollingBuffer {
 public:
  void push(float value) {
    data_[next_index_] = value;
    next_index_ = (next_index_ + 1) % Capacity;
    if (size_ < Capacity) {
      ++size_;
    }
  }

  [[nodiscard]] bool empty() const { return size_ == 0; }

  [[nodiscard]] size_t size() const { return size_; }

  void copy(std::vector<float>& out) const {
    out.clear();
    out.reserve(size_);
    if (size_ == 0) {
      return;
    }
    if (size_ < Capacity) {
      out.insert(out.end(), data_.begin(), data_.begin() + static_cast<long>(size_));
      return;
    }
    out.insert(out.end(), data_.begin() + static_cast<long>(next_index_), data_.end());
    out.insert(out.end(), data_.begin(), data_.begin() + static_cast<long>(next_index_));
  }

 private:
  std::array<float, Capacity> data_{};
  size_t next_index_{0};
  size_t size_{0};
};

inline double metersToLatitude(double north_meters) {
  return kBaseLatitudeDeg + north_meters / kMetersPerDegreeLat;
}

inline double metersToLongitude(double east_meters) {
  const double cos_lat = std::cos(kBaseLatitudeDeg * std::numbers::pi / 180.0);
  const double meters_per_degree_lon =
      kMetersPerDegreeLat * std::max(0.1, cos_lat);
  return kBaseLongitudeDeg + east_meters / meters_per_degree_lon;
}

const char* CanModeToString(fsai::control::runtime::CanIface::Mode mode) {
  switch (mode) {
    case fsai::control::runtime::CanIface::Mode::kSimulation:
      return "Simulation";
    case fsai::control::runtime::CanIface::Mode::kFsAiApi:
      return "FS AI API";
  }
  return "Unknown";
}

const char* AsStateToString(fsai::sim::svcu::dbc::AsState state) {
  switch (state) {
    case fsai::sim::svcu::dbc::AsState::kOff:
      return "Off";
    case fsai::sim::svcu::dbc::AsState::kReady:
      return "Ready";
    case fsai::sim::svcu::dbc::AsState::kDriving:
      return "Driving";
    case fsai::sim::svcu::dbc::AsState::kEmergencyBrake:
      return "Emergency Brake";
    case fsai::sim::svcu::dbc::AsState::kFinished:
      return "Finished";
    case fsai::sim::svcu::dbc::AsState::kR2d:
      return "Ready to Drive";
  }
  return "Unknown";
}

const char* SteeringStatusToString(fsai::sim::svcu::dbc::SteeringStatus status) {
  switch (status) {
    case fsai::sim::svcu::dbc::SteeringStatus::kOff:
      return "Off";
    case fsai::sim::svcu::dbc::SteeringStatus::kActive:
      return "Active";
    case fsai::sim::svcu::dbc::SteeringStatus::kFault:
      return "Fault";
    case fsai::sim::svcu::dbc::SteeringStatus::kUnknown:
      return "Unknown";
  }
  return "Unknown";
}

const char* BrakeStatusToString(fsai::sim::svcu::dbc::BrakeStatus status) {
  switch (status) {
    case fsai::sim::svcu::dbc::BrakeStatus::kInitialising:
      return "Initialising";
    case fsai::sim::svcu::dbc::BrakeStatus::kReady:
      return "Ready";
    case fsai::sim::svcu::dbc::BrakeStatus::kShuttingDown:
      return "Shutting Down";
    case fsai::sim::svcu::dbc::BrakeStatus::kShutdownComplete:
      return "Shutdown Complete";
    case fsai::sim::svcu::dbc::BrakeStatus::kFault:
      return "Fault";
  }
  return "Unknown";
}

const char* EbsStatusToString(fsai::sim::svcu::dbc::EbsStatus status) {
  switch (status) {
    case fsai::sim::svcu::dbc::EbsStatus::kUnavailable:
      return "Unavailable";
    case fsai::sim::svcu::dbc::EbsStatus::kArmed:
      return "Armed";
    case fsai::sim::svcu::dbc::EbsStatus::kTriggered:
      return "Triggered";
  }
  return "Unknown";
}

ImVec4 ColorForFreshness(double age_s, bool valid) {
  if (!valid || !std::isfinite(age_s)) {
    return ImVec4(0.85f, 0.3f, 0.3f, 1.0f);
  }
  if (age_s < 0.25) {
    return ImVec4(0.2f, 0.75f, 0.2f, 1.0f);
  }
  if (age_s < 0.75) {
    return ImVec4(0.95f, 0.85f, 0.25f, 1.0f);
  }
  return ImVec4(0.9f, 0.45f, 0.25f, 1.0f);
}

template <typename T>
ImVec4 ColorForFreshness(const fsai::sim::app::RuntimeTelemetry::TimedSample<T>& sample) {
  return ColorForFreshness(sample.age_s, sample.valid);
}

template <typename T, typename Fn>
void WithFreshnessColor(const fsai::sim::app::RuntimeTelemetry::TimedSample<T>& sample,
                        Fn&& fn) {
  const ImVec4 color = ColorForFreshness(sample);
  ImGui::PushStyleColor(ImGuiCol_Text, color);
  fn();
  ImGui::PopStyleColor();
}

void DrawSimulationPanel(const fsai::sim::app::RuntimeTelemetry& telemetry) {
  static RollingBuffer<360> speed_history;
  static RollingBuffer<360> accel_long_history;
  static RollingBuffer<360> accel_lat_history;
  static RollingBuffer<360> yaw_rate_history;
  static std::vector<float> plot_buffer;

  speed_history.push(telemetry.physics.vehicle_speed_kph);
  accel_long_history.push(telemetry.acceleration.longitudinal_mps2);
  accel_lat_history.push(telemetry.acceleration.lateral_mps2);
  yaw_rate_history.push(telemetry.acceleration.yaw_rate_degps);

  ImGui::Begin("Simulation Stats");

  ImGui::Text("Runtime mode: %s",
              telemetry.mode.runtime_mode.empty() ? "N/A"
                                                  : telemetry.mode.runtime_mode.c_str());
  ImGui::Text("AI command stream: %s",
              telemetry.control.ai_command_enabled ? "enabled" : "disabled");
  const char* ai_status = telemetry.control.ai_command_applied
                              ? "active"
                              : (telemetry.control.ai_command_enabled ? "waiting" : "inactive");
  ImGui::Text("AI control state: %s", ai_status);
  ImGui::Text("Simulation time: %.2f s", telemetry.physics.simulation_time_s);

  if (ImGui::CollapsingHeader("Vehicle Pose", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Position XYZ [m]: %.2f, %.2f, %.2f", telemetry.pose.position_x_m,
                telemetry.pose.position_y_m, telemetry.pose.position_z_m);
    ImGui::Text("Yaw: %.1f deg", telemetry.pose.yaw_deg);
  }

  if (ImGui::CollapsingHeader("Vehicle Kinematics", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Speed: %.2f m/s (%.1f km/h)", telemetry.physics.vehicle_speed_mps,
                telemetry.physics.vehicle_speed_kph);
    ImGui::Text("Steering: %.1f deg", telemetry.physics.steering_deg);
    ImGui::Text("Throttle: %.2f", telemetry.control.applied_throttle);
    ImGui::Text("Brake: %.2f", telemetry.control.applied_brake);
    if (telemetry.control.has_last_command && telemetry.control.last_command) {
      ImGui::Text("Last torque request F/R [Nm]: %.0f / %.0f",
                  telemetry.control.last_command->front_drive.axle_torque_request_nm,
                  telemetry.control.last_command->rear_drive.axle_torque_request_nm);
      ImGui::Text("Last brake request F/R [%%]: %.1f / %.1f",
                  telemetry.control.last_command->brake.front_pct,
                  telemetry.control.last_command->brake.rear_pct);
    }
  }

  if (ImGui::CollapsingHeader("Axle Torques", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Front axle torque: %.0f Nm (drive force %.0f N)",
                telemetry.drive.front_axle_torque_nm,
                telemetry.drive.front_drive_force_n);
    ImGui::Text("Rear axle torque: %.0f Nm (drive force %.0f N)",
                telemetry.drive.rear_axle_torque_nm,
                telemetry.drive.rear_drive_force_n);
    ImGui::Text("Net longitudinal forces F/R [N]: %.0f / %.0f",
                telemetry.drive.front_net_force_n, telemetry.drive.rear_net_force_n);
    if ((telemetry.can.front_drive.valid &&
         telemetry.can.front_drive.value.axle_trq_request_nm != 0.0f) ||
        (telemetry.can.rear_drive.valid &&
         telemetry.can.rear_drive.value.axle_trq_request_nm != 0.0f)) {
      ImGui::Text("Requested axle torque F/R [Nm]: %.0f / %.0f",
                  telemetry.can.front_drive.valid
                      ? telemetry.can.front_drive.value.axle_trq_request_nm
                      : 0.0f,
                  telemetry.can.rear_drive.valid
                      ? telemetry.can.rear_drive.value.axle_trq_request_nm
                      : 0.0f);
    }
  }

  if (ImGui::CollapsingHeader("Brake System", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Front brake force: %.0f N (%.1f%%)", telemetry.brake.front_force_n,
                telemetry.brake.front_pct);
    ImGui::Text("Rear brake force: %.0f N (%.1f%%)", telemetry.brake.rear_force_n,
                telemetry.brake.rear_pct);
    if (telemetry.can.brake.valid &&
        (telemetry.can.brake.value.front_req_pct != 0.0f ||
         telemetry.can.brake.value.rear_req_pct != 0.0f)) {
      ImGui::Text("Requested brake F/R [%%]: %.1f / %.1f",
                  telemetry.can.brake.value.front_req_pct,
                  telemetry.can.brake.value.rear_req_pct);
    }
  }

  if (ImGui::CollapsingHeader("Wheel RPM", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("wheel_rpm", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_NoHostExtendX)) {
      static constexpr const char* kWheelLabels[4] = {"LF", "RF", "LR", "RR"};
      for (int i = 0; i < 4; ++i) {
        ImGui::TableSetupColumn(kWheelLabels[i], ImGuiTableColumnFlags_WidthStretch);
      }
      ImGui::TableHeadersRow();
      ImGui::TableNextRow();
      for (int i = 0; i < 4; ++i) {
        ImGui::TableSetColumnIndex(i);
        ImGui::Text("%.0f", telemetry.wheels.rpm[static_cast<size_t>(i)]);
      }
      ImGui::EndTable();
    }
  }

  if (ImGui::CollapsingHeader("Accelerations", ImGuiTreeNodeFlags_DefaultOpen)) {
    constexpr float kGravity = 9.80665f;
    const float lon_g = telemetry.acceleration.longitudinal_mps2 / kGravity;
    const float lat_g = telemetry.acceleration.lateral_mps2 / kGravity;
    ImGui::Text("Accel longitudinal: %.2f m/s^2 (%.2f g)",
                telemetry.acceleration.longitudinal_mps2, lon_g);
    ImGui::Text("Accel lateral: %.2f m/s^2 (%.2f g)",
                telemetry.acceleration.lateral_mps2, lat_g);
    ImGui::Text("Accel vertical: %.2f m/s^2", telemetry.acceleration.vertical_mps2);
    ImGui::Text("Yaw rate: %.2f deg/s", telemetry.acceleration.yaw_rate_degps);
    if (telemetry.can.imu) {
      const auto& imu = *telemetry.can.imu;
      ImGui::Text("IMU sample a_lon/a_lat [m/s^2]: %.2f / %.2f", imu.accel_longitudinal_mps2,
                  imu.accel_lateral_mps2);
      ImGui::Text("IMU yaw rate: %.2f deg/s", imu.yaw_rate_rps *
                                              fsai::sim::svcu::dbc::kRadToDeg);
    }
  }

  if (ImGui::CollapsingHeader("Lap Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Completed laps: %d", telemetry.lap.completed_laps);
    ImGui::Text("Current lap time: %.1f s", telemetry.lap.current_lap_time_s);
    ImGui::Text("Distance traveled: %.1f m", telemetry.lap.total_distance_m);
    if (telemetry.can.gps) {
      ImGui::Text("GPS speed: %.2f m/s", telemetry.can.gps->speed_mps);
    }
  }

  if (ImGui::CollapsingHeader("Trends")) {
    auto draw_plot = [&](const char* label, RollingBuffer<360>& buffer, float pad) {
      buffer.copy(plot_buffer);
      if (plot_buffer.empty()) {
        return;
      }
      const auto [min_it, max_it] = std::minmax_element(plot_buffer.begin(), plot_buffer.end());
      const float min_v = *min_it - pad;
      const float max_v = *max_it + pad;
      ImGui::PlotLines(label, plot_buffer.data(), static_cast<int>(plot_buffer.size()), 0,
                       nullptr, min_v, max_v, ImVec2(0.0f, 80.0f));
    };
    draw_plot("Speed [km/h]", speed_history, 2.0f);
    draw_plot("Accel lon [m/s^2]", accel_long_history, 0.5f);
    draw_plot("Accel lat [m/s^2]", accel_lat_history, 0.5f);
    draw_plot("Yaw rate [deg/s]", yaw_rate_history, 1.0f);
  }

  ImGui::End();
}

void DrawControlPanel(const fsai::sim::app::RuntimeTelemetry& telemetry) {
  ImGui::Begin("Control Panel");

  const bool ai_enabled = telemetry.control.ai_command_enabled;
  const bool ai_applied = telemetry.control.ai_command_applied;
  ImGui::Text("AI command stream: %s", ai_enabled ? "enabled" : "disabled");
  if (ai_applied) {
    ImGui::TextColored(ImVec4(0.2f, 0.75f, 0.2f, 1.0f), "AI command applied");
  } else if (ai_enabled) {
    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.25f, 1.0f),
                       "Waiting for valid AI command (failsafe braking)");
  } else {
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "AI control disabled");
  }

  ImGui::Separator();
  ImGui::Text("AI request");
  if (telemetry.control.ai_command.valid) {
    const ImVec4 ai_color = ColorForFreshness(
        telemetry.control.ai_command.age_s,
        telemetry.control.ai_command.valid && ai_enabled);
    const float ai_steer_deg =
        telemetry.control.ai_command.value.steer_rad *
        fsai::sim::svcu::dbc::kRadToDeg;
    ImGui::TextColored(ai_color,
                       "Throttle: %.2f | Brake: %.2f | Steer: %.1f deg",
                       telemetry.control.ai_command.value.throttle,
                       telemetry.control.ai_command.value.brake, ai_steer_deg);
  } else {
    const ImVec4 stale_color =
        ColorForFreshness(std::numeric_limits<double>::infinity(), false);
    ImGui::TextColored(stale_color, "No AI command received");
  }

  ImGui::Separator();
  ImGui::Text("Applied command");
  ImGui::Text("Throttle: %.2f | Brake: %.2f | Steer: %.1f deg",
              telemetry.control.applied_throttle,
              telemetry.control.applied_brake,
              telemetry.control.applied_steer_rad *
                  fsai::sim::svcu::dbc::kRadToDeg);
  ImGui::Text("Source: %s", ai_applied ? "AI command" : "Failsafe");

  if (telemetry.control.has_last_command && telemetry.control.last_command) {
    const auto& last = *telemetry.control.last_command;
    ImGui::Separator();
    ImGui::Text("Last CAN command");
    ImGui::Text("Steer target: %.1f deg | Throttle: %.0f %% | Brake: %.0f %%",
                last.steer.steer_deg, last.throttle_clamped * 100.0f,
                last.brake_clamped * 100.0f);
    ImGui::Text("Torque request F/R: %.0f / %.0f Nm",
                last.front_drive.axle_torque_request_nm,
                last.rear_drive.axle_torque_request_nm);
  }

  if (ImGui::CollapsingHeader("Staleness timers", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (telemetry.control.ai_command.valid) {
      const ImVec4 color = ColorForFreshness(
          telemetry.control.ai_command.age_s,
          telemetry.control.ai_command.valid && ai_enabled);
      ImGui::TextColored(color, "AI request age: %.0f ms",
                         telemetry.control.ai_command.age_s * 1000.0);
    } else {
      const ImVec4 color =
          ColorForFreshness(std::numeric_limits<double>::infinity(), false);
      ImGui::TextColored(color, "AI request age: no data");
    }

    auto draw_ack_age = [&](const char* label, const auto& sample) {
      WithFreshnessColor(sample, [&]() {
        if (!sample.valid) {
          ImGui::Text("%s ack age: no data", label);
        } else {
          ImGui::Text("%s ack age: %.0f ms", label, sample.age_s * 1000.0);
        }
      });
    };

    draw_ack_age("Steer", telemetry.can.steer);
    draw_ack_age("Front drive", telemetry.can.front_drive);
    draw_ack_age("Rear drive", telemetry.can.rear_drive);
    draw_ack_age("Brake", telemetry.can.brake);
  }

  const auto ack_lagging = [](const auto& sample) {
    return !sample.valid || sample.age_s > kAckLagWarningSeconds;
  };
  const bool ack_delayed = ack_lagging(telemetry.can.steer) ||
                           ack_lagging(telemetry.can.front_drive) ||
                           ack_lagging(telemetry.can.rear_drive) ||
                           ack_lagging(telemetry.can.brake);

  if (ack_delayed) {
    ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f),
                       "CAN acknowledgements lagging");
  } else {
    ImGui::TextColored(ImVec4(0.2f, 0.75f, 0.2f, 1.0f),
                       "CAN acknowledgements healthy");
  }

  ImGui::End();
}

void DrawCanPanel(const fsai::sim::app::RuntimeTelemetry& telemetry) {
  ImGui::Begin("CAN Telemetry");

  ImGui::Text("Transport: %s", CanModeToString(telemetry.can.mode));
  ImGui::Text("Endpoint: %s",
              telemetry.can.endpoint.empty() ? "N/A" : telemetry.can.endpoint.c_str());

  const bool heartbeat_valid = telemetry.can.status.valid &&
                               std::isfinite(telemetry.can.last_heartbeat_age_s);
  const ImVec4 heartbeat_color =
      ColorForFreshness(telemetry.can.last_heartbeat_age_s, heartbeat_valid);
  if (heartbeat_valid) {
    ImGui::TextColored(heartbeat_color, "Heartbeat age: %.0f ms",
                       telemetry.can.last_heartbeat_age_s * 1000.0);
  } else {
    ImGui::TextColored(heartbeat_color, "Heartbeat age: no data");
  }

  if (ImGui::CollapsingHeader("VCU Status", ImGuiTreeNodeFlags_DefaultOpen)) {
    WithFreshnessColor(telemetry.can.status, [&]() {
      if (!telemetry.can.status.valid) {
        ImGui::Text("No status frames received");
        return;
      }
      const auto& status = telemetry.can.status.value;
      ImGui::Text("Handshake: %s | Go signal: %s",
                  status.handshake ? "Yes" : "No",
                  status.go_signal ? "Yes" : "No");
      ImGui::Text("AS state: %s | Steering: %s", AsStateToString(status.as_state),
                  SteeringStatusToString(status.steering_status));
      ImGui::Text("TS switch: %s | AS switch: %s",
                  status.ts_switch_on ? "On" : "Off",
                  status.as_switch_on ? "On" : "Off");
      ImGui::Text("Shutdown request: %s | Fault: %s | Warning: %s",
                  status.shutdown_request ? "Yes" : "No",
                  status.fault ? "Yes" : "No",
                  status.warning ? "Yes" : "No");
      const struct {
        bool flag;
        const char* label;
      } flagged_states[] = {
          {status.ai_estop_request, "AI e-stop request"},
          {status.hvil_open_fault, "HVIL open fault"},
          {status.hvil_short_fault, "HVIL short fault"},
          {status.ebs_fault, "EBS fault"},
          {status.offboard_charger_fault, "Offboard charger fault"},
          {status.ai_comms_lost, "AI communications lost"},
          {status.warn_batt_temp_high, "Battery temperature high"},
          {status.warn_batt_soc_low, "Battery SOC low"},
          {status.charge_procedure_fault, "Charge procedure fault"},
          {status.autonomous_braking_fault, "Autonomous braking fault"},
          {status.mission_status_fault, "Mission status fault"},
          {status.bms_fault, "BMS fault"},
          {status.brake_plausibility_fault, "Brake plausibility fault"},
      };
      bool any_flags = false;
      for (const auto& flag : flagged_states) {
        if (flag.flag) {
          if (!any_flags) {
            ImGui::Text("Active flags:");
            any_flags = true;
          }
          ImGui::BulletText("%s", flag.label);
        }
      }
      if (!any_flags) {
        ImGui::Text("Active flags: none");
      }
      ImGui::Text("Age: %.0f ms", telemetry.can.status.age_s * 1000.0);
    });
  }

  if (ImGui::CollapsingHeader("Steering Feedback", ImGuiTreeNodeFlags_DefaultOpen)) {
    WithFreshnessColor(telemetry.can.steer, [&]() {
      if (!telemetry.can.steer.valid) {
        ImGui::Text("No steering frames received");
        return;
      }
      const auto& steer = telemetry.can.steer.value;
      ImGui::Text("Angle: %.1f deg (req %.1f / max %.1f)", steer.angle_deg,
                  steer.angle_request_deg, steer.angle_max_deg);
      ImGui::Text("Age: %.0f ms", telemetry.can.steer.age_s * 1000.0);
    });
  }

  if (ImGui::CollapsingHeader("Drive Feedback", ImGuiTreeNodeFlags_DefaultOpen)) {
    auto draw_drive = [&](const char* label,
                          const fsai::sim::app::RuntimeTelemetry::TimedSample<
                              fsai::sim::svcu::dbc::Vcu2AiDrive>& sample) {
      WithFreshnessColor(sample, [&]() {
        if (!sample.valid) {
          ImGui::Text("%s: no drive frames", label);
          return;
        }
        const auto& drive = sample.value;
        ImGui::Text("%s axle torque: %.0f Nm", label, drive.axle_trq_nm);
        ImGui::Text("%s request / max: %.0f / %.0f Nm", label,
                    drive.axle_trq_request_nm, drive.axle_trq_max_nm);
        ImGui::Text("Age: %.0f ms", sample.age_s * 1000.0);
      });
    };
    draw_drive("Front", telemetry.can.front_drive);
    draw_drive("Rear", telemetry.can.rear_drive);
  }

  if (ImGui::CollapsingHeader("Brake Feedback", ImGuiTreeNodeFlags_DefaultOpen)) {
    WithFreshnessColor(telemetry.can.brake, [&]() {
      if (!telemetry.can.brake.valid) {
        ImGui::Text("No brake frames received");
        return;
      }
      const auto& brake = telemetry.can.brake.value;
      ImGui::Text("Actual F/R: %.1f / %.1f %%", brake.front_pct, brake.rear_pct);
      ImGui::Text("Requested F/R: %.1f / %.1f %%", brake.front_req_pct,
                  brake.rear_req_pct);
      ImGui::Text("Status: %s | EBS: %s", BrakeStatusToString(brake.status_brk),
                  EbsStatusToString(brake.status_ebs));
      ImGui::Text("Age: %.0f ms", telemetry.can.brake.age_s * 1000.0);
    });
  }

  if (ImGui::CollapsingHeader("Wheel Speeds", ImGuiTreeNodeFlags_DefaultOpen)) {
    WithFreshnessColor(telemetry.can.speeds, [&]() {
      if (!telemetry.can.speeds.valid) {
        ImGui::Text("No wheel speed frames received");
        return;
      }
      if (ImGui::BeginTable("can_wheel_rpm", 4,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                ImGuiTableFlags_NoHostExtendX)) {
        static constexpr const char* kWheelLabels[4] = {"LF", "RF", "LR", "RR"};
        for (int i = 0; i < 4; ++i) {
          ImGui::TableSetupColumn(kWheelLabels[i], ImGuiTableColumnFlags_WidthStretch);
        }
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();
        for (int i = 0; i < 4; ++i) {
          ImGui::TableSetColumnIndex(i);
          ImGui::Text("%.0f", telemetry.can.speeds.value.wheel_rpm[static_cast<size_t>(i)]);
        }
        ImGui::EndTable();
      }
      ImGui::Text("Age: %.0f ms", telemetry.can.speeds.age_s * 1000.0);
    });
  }

  if (ImGui::CollapsingHeader("Dynamics", ImGuiTreeNodeFlags_DefaultOpen)) {
    WithFreshnessColor(telemetry.can.dynamics, [&]() {
      if (!telemetry.can.dynamics.valid) {
        ImGui::Text("No dynamics frames received");
        return;
      }
      const auto& dyn = telemetry.can.dynamics.value;
      ImGui::Text("Speed actual/target: %.1f / %.1f km/h", dyn.speed_actual_kph,
                  dyn.speed_target_kph);
      ImGui::Text("Steer actual/target: %.1f / %.1f deg", dyn.steer_actual_deg,
                  dyn.steer_target_deg);
      ImGui::Text("Brake actual/target: %.1f / %.1f %%", dyn.brake_actual_pct,
                  dyn.brake_target_pct);
      ImGui::Text("Drive torque actual/target: %.1f / %.1f %%", dyn.drive_trq_actual_pct,
                  dyn.drive_trq_target_pct);
      ImGui::Text("Age: %.0f ms", telemetry.can.dynamics.age_s * 1000.0);
    });
  }

  ImGui::End();
}

ImVec4 ColorForLogLevel(fsai::sim::log::Level level) {
  switch (level) {
    case fsai::sim::log::Level::kInfo:
      return ImVec4(0.9f, 0.9f, 0.9f, 1.0f);
    case fsai::sim::log::Level::kWarning:
      return ImVec4(0.95f, 0.85f, 0.3f, 1.0f);
    case fsai::sim::log::Level::kError:
      return ImVec4(0.95f, 0.4f, 0.4f, 1.0f);
  }
  return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
}

void DrawLogConsole() {
  ImGui::Begin("Simulation Log");

  if (ImGui::Button("Clear")) {
    fsai::sim::log::Clear();
  }
  ImGui::SameLine();
  static bool auto_scroll = true;
  ImGui::Checkbox("Auto-scroll", &auto_scroll);

  ImGui::Separator();

  const auto entries = fsai::sim::log::Snapshot();
  const bool should_scroll = fsai::sim::log::ConsumeScrollRequest();

  ImGui::BeginChild("LogConsoleScroll", ImVec2(0, 0), false,
                    ImGuiWindowFlags_HorizontalScrollbar);
  for (const auto& entry : entries) {
    ImGui::PushStyleColor(ImGuiCol_Text, ColorForLogLevel(entry.level));
    ImGui::TextUnformatted(entry.formatted.c_str());
    ImGui::PopStyleColor();
  }
  if (auto_scroll && should_scroll) {
    ImGui::SetScrollHereY(1.0f);
  }
  ImGui::EndChild();

  ImGui::End();
}

void DrawWorldScene(Graphics* graphics, const World& world,
                    const fsai::sim::app::RuntimeTelemetry& telemetry) {
  (void)telemetry;
  fsai::time::SimulationStageTimer render_timer("renderer");
  Graphics_Clear(graphics);
  Graphics_DrawGrid(graphics, 50);

  const auto& checkpoints = world.checkpointPositionsWorld();
  if (!checkpoints.empty()) {
    SDL_SetRenderDrawColor(graphics->renderer, 200, 0, 200, 255);
    Graphics_DrawFilledCircle(
        graphics,
        static_cast<int>(checkpoints.front().x * kRenderScale +
                         graphics->width / 2.0f),
        static_cast<int>(checkpoints.front().z * kRenderScale +
                         graphics->height / 2.0f),
        static_cast<int>(1.5f * kRenderScale));
  }

  const auto& lookahead = world.lookahead();
  SDL_SetRenderDrawColor(graphics->renderer, 120, 120, 120, 255);
  for (size_t i = 0; i < world.leftConePositions().size(); ++i) {
    if (i == 0) {
      SDL_SetRenderDrawColor(graphics->renderer, 0, 255, 0, 255);
    } else if (static_cast<int>(i) == lookahead.speed) {
      SDL_SetRenderDrawColor(graphics->renderer, 255, 255, 0, 255);
    } else if (static_cast<int>(i) == lookahead.steer) {
      SDL_SetRenderDrawColor(graphics->renderer, 255, 0, 255, 255);
    } else {
      SDL_SetRenderDrawColor(graphics->renderer, 120, 120, 120, 255);
    }
    const auto& cone = world.leftConePositions()[i];
    const int cone_x = static_cast<int>(cone.x * kRenderScale +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * kRenderScale +
                                        graphics->height / 2.0f);
    Graphics_DrawFilledCircle(graphics, cone_x, cone_y,
                              static_cast<int>(kRenderScale));
  }

  for (size_t i = 0; i < world.rightConePositions().size(); ++i) {
    if (i == 0) {
      SDL_SetRenderDrawColor(graphics->renderer, 0, 255, 0, 255);
    } else if (static_cast<int>(i) == lookahead.speed) {
      SDL_SetRenderDrawColor(graphics->renderer, 255, 255, 0, 255);
    } else if (static_cast<int>(i) == lookahead.steer) {
      SDL_SetRenderDrawColor(graphics->renderer, 255, 0, 255, 255);
    } else {
      SDL_SetRenderDrawColor(graphics->renderer, 80, 80, 80, 255);
    }
    const auto& cone = world.rightConePositions()[i];
    const int cone_x = static_cast<int>(cone.x * kRenderScale +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * kRenderScale +
                                        graphics->height / 2.0f);
    Graphics_DrawFilledCircle(graphics, cone_x, cone_y,
                              static_cast<int>(kRenderScale));
  }

  const auto& transform = world.vehicleTransform();
  const float car_screen_x = transform.position.x * kRenderScale +
                             graphics->width / 2.0f;
  const float car_screen_y = transform.position.z * kRenderScale +
                             graphics->height / 2.0f;
  const float car_radius = 2.0f * kRenderScale;
  Graphics_DrawCar(graphics, car_screen_x, car_screen_y, car_radius,
                   transform.yaw);
}

struct ChannelNoiseConfig {
  double noise_std{0.0};
  double latency_s{0.0};
};

struct TorqueNoiseConfig {
  double front_noise_std_nm{0.0};
  double rear_noise_std_nm{0.0};
  double latency_s{0.0};
};

struct ImuNoiseConfig {
  double accel_longitudinal_std{0.0};
  double accel_lateral_std{0.0};
  double yaw_rate_std_degps{0.0};
  double latency_s{0.0};
};

struct GpsNoiseConfig {
  double lat_std_deg{0.0};
  double lon_std_deg{0.0};
  double speed_std_mps{0.0};
  double latency_s{0.0};
};

struct SensorNoiseConfig {
  uint32_t seed{1337u};
  ChannelNoiseConfig steering_deg{};
  ChannelNoiseConfig wheel_rpm{};
  TorqueNoiseConfig drive_torque{};
  ChannelNoiseConfig brake_pct{};
  ChannelNoiseConfig speed_kph{};
  ImuNoiseConfig imu{};
  GpsNoiseConfig gps{};
};

template <typename T>
class SensorDelayLine {
 public:
  void set_latency(uint64_t latency_ns) { latency_ns_ = latency_ns; }

  void push(uint64_t timestamp_ns, const T& value) {
    queue_.emplace_back(timestamp_ns, value);
  }

  std::optional<T> poll(uint64_t now_ns) {
    if (latency_ns_ == 0) {
      if (!queue_.empty()) {
        last_value_ = queue_.back().second;
        queue_.clear();
      }
      return last_value_;
    }
    while (!queue_.empty()) {
      const auto& front = queue_.front();
      if (front.first + latency_ns_ <= now_ns) {
        last_value_ = front.second;
        queue_.pop_front();
      } else {
        break;
      }
    }
    return last_value_;
  }

 private:
  uint64_t latency_ns_{0};
  std::deque<std::pair<uint64_t, T>> queue_{};
  std::optional<T> last_value_{};
};

struct TorqueSample {
  float front_nm{0.0f};
  float rear_nm{0.0f};
};

struct GpsSample {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double speed_mps{0.0};
};

SensorNoiseConfig LoadSensorNoiseConfig(const std::string& path) {
  SensorNoiseConfig cfg;
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (auto seed = root["seed"]) {
      cfg.seed = seed.as<uint32_t>(cfg.seed);
    }
    if (auto steering = root["steering"]) {
      cfg.steering_deg.noise_std = steering["noise_std_deg"].as<double>(cfg.steering_deg.noise_std);
      if (auto latency = steering["latency_ms"]) {
        cfg.steering_deg.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto wheel = root["wheel_speed"]) {
      cfg.wheel_rpm.noise_std = wheel["noise_std_rpm"].as<double>(cfg.wheel_rpm.noise_std);
      if (auto latency = wheel["latency_ms"]) {
        cfg.wheel_rpm.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto torque = root["drive_torque"]) {
      cfg.drive_torque.front_noise_std_nm = torque["front_noise_std_nm"].as<double>(cfg.drive_torque.front_noise_std_nm);
      cfg.drive_torque.rear_noise_std_nm = torque["rear_noise_std_nm"].as<double>(cfg.drive_torque.rear_noise_std_nm);
      if (auto latency = torque["latency_ms"]) {
        cfg.drive_torque.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto brake = root["brake"]) {
      cfg.brake_pct.noise_std = brake["noise_std_pct"].as<double>(cfg.brake_pct.noise_std);
      if (auto latency = brake["latency_ms"]) {
        cfg.brake_pct.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto speed = root["speed"]) {
      cfg.speed_kph.noise_std = speed["noise_std_kph"].as<double>(cfg.speed_kph.noise_std);
      if (auto latency = speed["latency_ms"]) {
        cfg.speed_kph.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto imu = root["imu"]) {
      cfg.imu.accel_longitudinal_std = imu["noise_std_ax"].as<double>(cfg.imu.accel_longitudinal_std);
      cfg.imu.accel_lateral_std = imu["noise_std_ay"].as<double>(cfg.imu.accel_lateral_std);
      cfg.imu.yaw_rate_std_degps = imu["noise_std_yaw_degps"].as<double>(cfg.imu.yaw_rate_std_degps);
      if (auto latency = imu["latency_ms"]) {
        cfg.imu.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
    if (auto gps = root["gps"]) {
      cfg.gps.lat_std_deg = gps["noise_std_lat_deg"].as<double>(cfg.gps.lat_std_deg);
      cfg.gps.lon_std_deg = gps["noise_std_lon_deg"].as<double>(cfg.gps.lon_std_deg);
      cfg.gps.speed_std_mps = gps["noise_std_speed_mps"].as<double>(cfg.gps.speed_std_mps);
      if (auto latency = gps["latency_ms"]) {
        cfg.gps.latency_s = latency.as<double>(0.0) * 1e-3;
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Failed to load sensor config '%s': %s\n", path.c_str(), e.what());
  }
  return cfg;
}
}  // namespace

int main(int argc, char* argv[]) {
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  auto parse_port = [](const char* text, uint16_t fallback) {
    if (!text) return fallback;
    char* end = nullptr;
    long value = std::strtol(text, &end, 10);
    if (end == text || value < 0 || value > 65535) {
      return fallback;
    }
    return static_cast<uint16_t>(value);
  };

  double dt = kDefaultDt;
  std::string can_iface = fsai::io::can::DefaultEndpoint();
  std::string mode = "sim";
  bool can_iface_overridden = false;
  uint16_t command_port = kDefaultCommandPort;
  uint16_t telemetry_port = kDefaultTelemetryPort;
  std::string sensor_config_path = kDefaultSensorConfig;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--dt" && i + 1 < argc) {
      dt = std::atof(argv[++i]);
    } else if (arg == "--can-if" && i + 1 < argc) {
      can_iface = argv[++i];
      can_iface_overridden = true;
    } else if (arg == "--mode" && i + 1 < argc) {
      mode = argv[++i];
    } else if (arg == "--cmd-port" && i + 1 < argc) {
      command_port = parse_port(argv[++i], command_port);
    } else if (arg == "--state-port" && i + 1 < argc) {
      telemetry_port = parse_port(argv[++i], telemetry_port);
    } else if (arg == "--sensor-config" && i + 1 < argc) {
      sensor_config_path = argv[++i];
    } else if (arg == "--help") {
      fsai::sim::log::LogfToStdout(
          fsai::sim::log::Level::kInfo,
          "Usage: fsai_run [--dt seconds] [--can-if iface|udp:port] [--cmd-port p] "
          "[--state-port p] [--sensor-config path]");
      return EXIT_SUCCESS;
    } else if (i == 1 && argc == 2) {
      dt = std::atof(arg.c_str());
    } else {
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Ignoring unrecognized argument '%s'", arg.c_str());
    }
  }

  if (dt <= 0.0) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "Invalid dt value provided. Using default dt = %.3f",
                         kDefaultDt);
    dt = kDefaultDt;
  }
  if (!can_iface_overridden) {
    if (mode == "sim") {
      can_iface = "vcan0";
    } else if (mode == "car") {
      can_iface = "can0";
    }
  }
  can_iface = fsai::io::can::CanonicalizeEndpoint(can_iface);
  const bool can_is_udp = fsai::io::can::IsUdpEndpoint(can_iface);
  fsai::sim::log::Logf(
      fsai::sim::log::Level::kInfo,
      "Using dt = %.4f seconds, mode=%s, CAN %s (%s), cmd-port %u, state-port %u",
      dt, mode.c_str(), can_iface.c_str(), can_is_udp ? "udp" : "socketcan",
      command_port, telemetry_port);

  SensorNoiseConfig sensor_cfg = LoadSensorNoiseConfig(sensor_config_path);
  std::mt19937 noise_rng(sensor_cfg.seed);
  auto sample_noise = [&noise_rng](double stddev) {
    if (stddev <= 0.0) {
      return 0.0;
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(noise_rng);
  };

  fsai_clock_config clock_cfg{};
  clock_cfg.mode = FSAI_CLOCK_MODE_SIMULATED;
  clock_cfg.start_time_ns = 0;
  fsai_clock_init(clock_cfg);

  const uint64_t step_ns = fsai_clock_from_seconds(dt);
  const double step_seconds = fsai_clock_to_seconds(step_ns);
  const uint64_t ai2vcu_period_ns = fsai_clock_from_seconds(0.01);
  uint64_t last_ai2vcu_tx_ns = 0;

  SensorDelayLine<float> steer_delay;
  SensorDelayLine<std::array<float, 4>> wheel_delay;
  SensorDelayLine<TorqueSample> torque_delay;
  SensorDelayLine<fsai::sim::svcu::dbc::Ai2LogDynamics2> imu_delay;
  SensorDelayLine<GpsSample> gps_delay;

  steer_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.steering_deg.latency_s));
  wheel_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.wheel_rpm.latency_s));
  torque_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.drive_torque.latency_s));
  imu_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.imu.latency_s));
  gps_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.gps.latency_s));

  float steer_meas_deg = 0.0f;
  bool has_steer_meas = false;
  std::array<float, 4> wheel_meas_rpm{0.0f, 0.0f, 0.0f, 0.0f};
  bool has_wheel_meas = false;
  TorqueSample torque_meas{};
  bool has_torque_meas = false;
  fsai::sim::svcu::dbc::Ai2LogDynamics2 imu_meas{};
  bool has_imu_meas = false;
  GpsSample gps_meas{};
  bool has_gps_meas = false;

  fsai_budget_init();
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_SIMULATION, "Simulation Renderer",
                        fsai_clock_from_seconds(1.0 / 60.0));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CONTROL, "Planner + Controller",
                        fsai_clock_from_seconds(0.005));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_VISION, "Vision Pipeline",
                        fsai_clock_from_seconds(0.020));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CAN, "CAN Dispatch",
                        fsai_clock_from_seconds(0.002));
  fsai_budget_mark_unimplemented(
      FSAI_BUDGET_SUBSYSTEM_VISION,
      "Vision pipeline not yet integrated; timing scaffold active.");
  fsai_budget_mark_unimplemented(
      FSAI_BUDGET_SUBSYSTEM_CAN,
      "CAN transport not wired; pending hardware integration.");

  const std::string vehicle_config_path = "../configs/vehicle/configDry.yaml";

  World world;
  world.init(vehicle_config_path.c_str());

  Graphics graphics{};
  if (Graphics_Init(&graphics, "Car Simulation 2D", kWindowWidth,
                    kWindowHeight) != 0) {
    std::fprintf(stderr, "Graphics_Init failed\n");
    return EXIT_FAILURE;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();
  if (!ImGui_ImplSDL2_InitForSDLRenderer(graphics.window, graphics.renderer)) {
    std::fprintf(stderr, "ImGui_ImplSDL2_InitForSDLRenderer failed\n");
    ImGui::DestroyContext();
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }
  if (!ImGui_ImplSDLRenderer2_Init(graphics.renderer)) {
    std::fprintf(stderr, "ImGui_ImplSDLRenderer2_Init failed\n");
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }

  auto shutdown_imgui = [&]() {
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
  };

  fsai::sim::integration::CsvLogger logger("CarStateLog.csv", "RALog.csv");
  if (!logger.valid()) {
    std::fprintf(stderr, "Failed to open CSV logs\n");
    shutdown_imgui();
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }

  fsai::sim::integration::registerBuiltInStereoProviders();
  auto stereo_factory =
      fsai::sim::integration::lookupStereoProvider("sim_stereo");
  std::unique_ptr<fsai::io::camera::sim_stereo::SimStereoSource> stereo_source;
  if (stereo_factory) {
    stereo_source = stereo_factory();
  }
  fsai::sim::integration::StereoDisplay stereo_display;

  std::vector<std::array<float, 3>> cone_positions;
  bool running = true;
  size_t frame_counter = 0;

  const VehicleParam& vehicle_param = world.model().param();

  fsai::control::runtime::CanIface::Config can_cfg{};
  can_cfg.endpoint = can_iface;
  can_cfg.enable_loopback = true;
  can_cfg.mode = (mode == "car") ? fsai::control::runtime::CanIface::Mode::kFsAiApi
                                   : fsai::control::runtime::CanIface::Mode::kSimulation;
  if (can_cfg.mode == fsai::control::runtime::CanIface::Mode::kFsAiApi) {
    can_cfg.enable_loopback = false;
  }
  can_cfg.wheel_radius_m = vehicle_param.tire.radius;

  fsai::control::runtime::CanIface can_interface;
  if (!can_interface.Initialize(can_cfg)) {
    if (mode == "sim" && !can_iface_overridden) {
      const std::string fallback = fsai::io::can::DefaultEndpoint();
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Falling back to CAN endpoint %s", fallback.c_str());
      can_cfg.endpoint = fallback;
      can_cfg.mode = fsai::control::runtime::CanIface::Mode::kSimulation;
      if (!can_interface.Initialize(can_cfg)) {
        std::fprintf(stderr, "Failed to open CAN endpoint %s\n", can_cfg.endpoint.c_str());
        shutdown_imgui();
        Graphics_Cleanup(&graphics);
        return EXIT_FAILURE;
      }
    } else {
      std::fprintf(stderr, "Failed to open CAN endpoint %s\n", can_cfg.endpoint.c_str());
      shutdown_imgui();
      Graphics_Cleanup(&graphics);
      return EXIT_FAILURE;
    }
  }

  fsai::sim::svcu::UdpEndpoint command_rx;
  if (!command_rx.bind(command_port)) {
    std::fprintf(stderr, "Failed to bind command UDP port %u\n", command_port);
    shutdown_imgui();
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }

  fsai::sim::svcu::UdpEndpoint telemetry_tx;
  if (!telemetry_tx.connect(telemetry_port)) {
    std::fprintf(stderr, "Failed to connect telemetry UDP port %u\n",
                 telemetry_port);
    shutdown_imgui();
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }

  float tmp_brake_front_bias = static_cast<float>(vehicle_param.brakes.front_bias);
  float tmp_brake_rear_bias = static_cast<float>(vehicle_param.brakes.rear_bias);
  const float brake_sum = tmp_brake_front_bias + tmp_brake_rear_bias;
  if (brake_sum > 0.0f) {
    tmp_brake_front_bias = std::clamp(tmp_brake_front_bias / brake_sum, 0.0f, 1.0f);
  } else {
    tmp_brake_front_bias = 0.5f;
  }
  const float brake_front_bias = tmp_brake_front_bias;
  const float brake_rear_bias = std::clamp(1.0f - brake_front_bias, 0.0f, 1.0f);

  fsai::control::runtime::Ai2VcuAdapterConfig adapter_cfg{};
  adapter_cfg.front_torque_fraction =
      static_cast<float>(vehicle_param.powertrain.torque_split_front);
  adapter_cfg.rear_torque_fraction =
      static_cast<float>(vehicle_param.powertrain.torque_split_rear);
  adapter_cfg.front_axle_max_torque_nm =
      static_cast<float>(vehicle_param.powertrain.torque_front_max_nm);
  adapter_cfg.rear_axle_max_torque_nm =
      static_cast<float>(vehicle_param.powertrain.torque_rear_max_nm);
  adapter_cfg.brake_front_bias = brake_front_bias;
  adapter_cfg.brake_rear_bias = brake_rear_bias;
  adapter_cfg.max_speed_kph =
      static_cast<float>(vehicle_param.input_ranges.vel.max * 3.6);
  fsai::control::runtime::Ai2VcuAdapter ai2vcu_adapter(adapter_cfg);

  const std::string svcu_endpoint =
      fsai::sim::svcu::canonicalize_can_endpoint(can_cfg.endpoint);
  fsai::sim::svcu::RunConfig svcu_config{};
  svcu_config.vehicle_config = vehicle_config_path;
  svcu_config.can_endpoint = svcu_endpoint;
  svcu_config.command_port = command_port;
  svcu_config.telemetry_port = telemetry_port;
  svcu_config.can_loopback = can_cfg.enable_loopback;

  std::atomic_bool svcu_keep_running{true};
  std::atomic_bool svcu_thread_active{true};
  std::optional<fsai::sim::svcu::RunStats> svcu_stats;
  std::atomic<uint64_t> svcu_last_command_ns{0};

  fsai::sim::svcu::RunCallbacks svcu_callbacks{};
  svcu_callbacks.on_command =
      [&svcu_last_command_ns](const fsai::sim::svcu::CommandSample& sample) {
        svcu_last_command_ns.store(sample.t_ns, std::memory_order_relaxed);
      };

  std::thread svcu_thread([svcu_config, &svcu_keep_running, &svcu_thread_active,
                           &svcu_stats, svcu_callbacks]() mutable {
    fsai::sim::svcu::RunLimits svcu_limits{};
    svcu_limits.stop_flag = &svcu_keep_running;
    svcu_limits.sleep_interval = std::chrono::milliseconds(1);
    const auto stats = fsai::sim::svcu::RunSvcu(svcu_config, svcu_limits,
                                                svcu_callbacks);
    svcu_stats = stats;
    svcu_thread_active.store(false, std::memory_order_release);
  });

  fsai::control::runtime::Ai2VcuCommandSet last_ai2vcu_commands{};
  bool has_last_ai_command = false;

  std::optional<fsai::sim::svcu::CommandPacket> latest_command;
  const uint64_t stale_threshold_ns =
      fsai_clock_from_seconds(kCommandStaleSeconds);


  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      Graphics_HandleWindowEvent(&graphics, &event);
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }
    if (!running) {
      break;
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    can_interface.Poll(fsai_clock_now());

    if (!svcu_thread_active.load(std::memory_order_acquire) &&
        svcu_keep_running.load(std::memory_order_relaxed)) {
      if (svcu_stats.has_value() && !svcu_stats->ok) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                             "SVCU control loop exited unexpectedly");
      } else {
        fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                             "SVCU control loop stopped");
      }
      running = false;
      continue;
    }

    const VehicleState& veh_state = world.vehicleState();
    const double vx = veh_state.velocity.x();
    const double vy = veh_state.velocity.y();
    const float speed_mps = static_cast<float>(std::sqrt(vx * vx + vy * vy));
    const auto can_vehicle_state = can_interface.LatestVehicleState();
    float measured_speed_mps = speed_mps;
    if (can_vehicle_state) {
      measured_speed_mps = std::max(can_vehicle_state->vx, measured_speed_mps);
    }

    float desiredThrottle = 0.0f;
    float desiredBrake = 0.0f;
    float desiredSteer = 0.0f;
    {
      float controller_throttle = 0.0f;
      float controller_steer = 0.0f;
      if (world.computeRacingControl(step_seconds, controller_throttle,
                                     controller_steer)) {
        desiredSteer = controller_steer;
        if (controller_throttle >= 0.0f) {
          desiredThrottle = controller_throttle;
          desiredBrake = 0.0f;
        } else {
          desiredThrottle = 0.0f;
          desiredBrake = -controller_throttle;
        }
      }
    }
    desiredThrottle = std::clamp(desiredThrottle, 0.0f, 1.0f);
    desiredBrake = std::clamp(desiredBrake, 0.0f, 1.0f);

    fsai::sim::svcu::CommandPacket command_packet{};
    while (auto bytes = command_rx.receive(&command_packet, sizeof(command_packet))) {
      if (*bytes == sizeof(command_packet)) {
        latest_command = command_packet;
      }
    }

    const uint64_t now_ns = fsai_clock_advance(step_ns);

    auto compute_age_seconds = [&](uint64_t timestamp_ns) {
      if (timestamp_ns == 0 || now_ns == 0 || now_ns < timestamp_ns) {
        return std::numeric_limits<double>::infinity();
      }
      return static_cast<double>(now_ns - timestamp_ns) * 1e-9;
    };

    float appliedThrottle = desiredThrottle;
    float appliedBrake = desiredBrake;
    float appliedSteer = desiredSteer;
    bool ai_command_applied = false;
    bool ai_command_enabled = false;

    fsai::sim::app::RuntimeTelemetry::TimedSample<fsai::types::ControlCmd>
        ai_command_sample{};
    ai_command_sample.age_s = std::numeric_limits<double>::infinity();

    if (latest_command) {
      ai_command_sample.valid = true;
      ai_command_sample.value.steer_rad = latest_command->steer_rad;
      ai_command_sample.value.throttle = latest_command->throttle;
      ai_command_sample.value.brake = latest_command->brake;
      ai_command_sample.value.t_ns = latest_command->t_ns;
      ai_command_sample.age_s = compute_age_seconds(latest_command->t_ns);

      ai_command_enabled = latest_command->enabled != 0;
      const bool command_fresh = latest_command->t_ns <= now_ns &&
                                 now_ns - latest_command->t_ns <= stale_threshold_ns;
      if (command_fresh) {
        appliedThrottle = latest_command->throttle;
        appliedBrake = latest_command->brake;
        appliedSteer = latest_command->steer_rad;
        ai_command_applied = true;
      } else if (ai_command_enabled) {
        appliedThrottle = 0.0f;
        appliedBrake = 1.0f;
        appliedSteer = 0.0f;
      } else {
        appliedThrottle = latest_command->throttle;
        appliedBrake = latest_command->brake;
        appliedSteer = latest_command->steer_rad;
      }
    } else {
      ai_command_enabled = true;
      ai_command_applied = true;
      ai_command_sample.valid = true;
      ai_command_sample.value.steer_rad = desiredSteer;
      ai_command_sample.value.throttle = desiredThrottle;
      ai_command_sample.value.brake = desiredBrake;
      ai_command_sample.value.t_ns = now_ns;
      ai_command_sample.age_s = 0.0;
    }

    appliedThrottle = std::clamp(appliedThrottle, 0.0f, 1.0f);
    appliedBrake = std::clamp(appliedBrake, 0.0f, 1.0f);

    fsai::types::ControlCmd ai_control_cmd{};
    ai_control_cmd.throttle = desiredThrottle;
    ai_control_cmd.brake = desiredBrake;
    ai_control_cmd.steer_rad = desiredSteer;
    ai_control_cmd.t_ns = now_ns;

    if (now_ns - last_ai2vcu_tx_ns >= ai2vcu_period_ns) {
      fsai::control::runtime::Ai2VcuAdapter::AdapterTelemetry adapter_telemetry{};
      adapter_telemetry.measured_speed_mps = measured_speed_mps;
      adapter_telemetry.lap_counter = static_cast<uint8_t>(
          std::clamp(world.completedLaps(), 0, 15));
      const auto total_cones = world.leftConePositions().size() +
                               world.rightConePositions().size();
      if (total_cones > 0) {
        adapter_telemetry.cones_count_all = static_cast<uint16_t>(
            std::min<size_t>(total_cones, std::numeric_limits<uint16_t>::max()));
      }

      auto command_frames = ai2vcu_adapter.Adapt(
          ai_control_cmd, can_interface.RawStatus(), adapter_telemetry);
      if (can_interface.Send(command_frames, now_ns)) {
        last_ai2vcu_tx_ns = now_ns;
        last_ai2vcu_commands = command_frames;
        has_last_ai_command = true;
      }
    }

    can_interface.Poll(now_ns);

    world.setSvcuCommand(appliedThrottle, appliedBrake, appliedSteer);
    world.throttleInput = appliedThrottle;
    world.brakeInput = appliedBrake;
    world.steeringAngle = appliedSteer;

    {
      fsai::time::ControlStageTimer control_timer("world_update");
      world.update(step_seconds);
    }
    const double sim_time_s = fsai_clock_to_seconds(now_ns);

    const auto& vehicle_state = world.vehicleState();
    const auto& model = world.model();
    const auto& pt_status = model.lastPowertrainStatus();
    const auto& brake_status = model.lastBrakeStatus();
    const WheelsInfo& wheel_info = world.wheelsInfo();
    const double wheel_radius = std::max(0.01, model.param().tire.radius);
    const double front_drive_force = pt_status.front_drive_force - pt_status.front_regen_force;
    const double rear_drive_force = pt_status.rear_drive_force - pt_status.rear_regen_force;
    const double front_net_force = front_drive_force - brake_status.front_force;
    const double rear_net_force = rear_drive_force - brake_status.rear_force;
    const double front_axle_torque_nm = front_drive_force * wheel_radius;
    const double rear_axle_torque_nm = rear_drive_force * wheel_radius;

    const double max_brake_force = std::max(1.0, model.param().brakes.max_force);
    const double front_max_force = std::max(1e-3, max_brake_force * adapter_cfg.brake_front_bias);
    const double rear_max_force = std::max(1e-3, max_brake_force * adapter_cfg.brake_rear_bias);
    const double front_brake_pct = front_max_force > 0.0
                                       ? (std::max(0.0, brake_status.front_force) / front_max_force) * 100.0
                                       : 0.0;
    const double rear_brake_pct = rear_max_force > 0.0
                                      ? (std::max(0.0, brake_status.rear_force) / rear_max_force) * 100.0
                                      : 0.0;

    const double vehicle_speed_mps = std::sqrt(
        vehicle_state.velocity.x() * vehicle_state.velocity.x() +
        vehicle_state.velocity.y() * vehicle_state.velocity.y());
    const float actual_speed_kph = static_cast<float>(vehicle_speed_mps * 3.6);
    const float actual_steer_deg = static_cast<float>(wheel_info.steering *
        fsai::sim::svcu::dbc::kRadToDeg);

    fsai::sim::app::RuntimeTelemetry runtime_telemetry{};
    runtime_telemetry.physics.simulation_time_s = sim_time_s;
    runtime_telemetry.physics.vehicle_speed_mps = static_cast<float>(vehicle_speed_mps);
    runtime_telemetry.physics.vehicle_speed_kph = actual_speed_kph;
    runtime_telemetry.physics.steering_deg = actual_steer_deg;
    runtime_telemetry.pose.position_x_m = static_cast<float>(vehicle_state.position.x());
    runtime_telemetry.pose.position_y_m = static_cast<float>(vehicle_state.position.y());
    runtime_telemetry.pose.position_z_m = static_cast<float>(vehicle_state.position.z());
    runtime_telemetry.pose.yaw_deg = static_cast<float>(vehicle_state.yaw *
                                                        fsai::sim::svcu::dbc::kRadToDeg);
    runtime_telemetry.wheels.rpm = {wheel_info.lf_speed, wheel_info.rf_speed,
                                    wheel_info.lb_speed, wheel_info.rb_speed};
    runtime_telemetry.drive.front_drive_force_n = static_cast<float>(front_drive_force);
    runtime_telemetry.drive.rear_drive_force_n = static_cast<float>(rear_drive_force);
    runtime_telemetry.drive.front_net_force_n = static_cast<float>(front_net_force);
    runtime_telemetry.drive.rear_net_force_n = static_cast<float>(rear_net_force);
    runtime_telemetry.drive.front_axle_torque_nm = static_cast<float>(front_axle_torque_nm);
    runtime_telemetry.drive.rear_axle_torque_nm = static_cast<float>(rear_axle_torque_nm);
    runtime_telemetry.brake.front_force_n = static_cast<float>(brake_status.front_force);
    runtime_telemetry.brake.rear_force_n = static_cast<float>(brake_status.rear_force);
    runtime_telemetry.brake.front_pct = static_cast<float>(front_brake_pct);
    runtime_telemetry.brake.rear_pct = static_cast<float>(rear_brake_pct);
    runtime_telemetry.acceleration.longitudinal_mps2 =
        static_cast<float>(vehicle_state.acceleration.x());
    runtime_telemetry.acceleration.lateral_mps2 =
        static_cast<float>(vehicle_state.acceleration.y());
    runtime_telemetry.acceleration.vertical_mps2 =
        static_cast<float>(vehicle_state.acceleration.z());
    runtime_telemetry.acceleration.yaw_rate_degps =
        static_cast<float>(vehicle_state.rotation.z() * fsai::sim::svcu::dbc::kRadToDeg);
    runtime_telemetry.lap.current_lap_time_s = world.lapTimeSeconds();
    runtime_telemetry.lap.total_distance_m = world.totalDistanceMeters();
    runtime_telemetry.lap.completed_laps = world.completedLaps();
    runtime_telemetry.can.mode = can_interface.mode();
    runtime_telemetry.can.endpoint = can_interface.endpoint();

    runtime_telemetry.can.status.valid = can_interface.HasStatus();
    if (runtime_telemetry.can.status.valid) {
      runtime_telemetry.can.status.value = can_interface.RawStatus();
      runtime_telemetry.can.status.age_s = compute_age_seconds(can_interface.LastStatusTimestampNs());
    } else {
      runtime_telemetry.can.status.age_s = std::numeric_limits<double>::infinity();
    }
    runtime_telemetry.can.last_heartbeat_age_s = runtime_telemetry.can.status.age_s;

    runtime_telemetry.can.steer.valid = can_interface.HasSteer();
    if (runtime_telemetry.can.steer.valid) {
      runtime_telemetry.can.steer.value = can_interface.RawSteer();
      runtime_telemetry.can.steer.age_s = compute_age_seconds(can_interface.LastSteerTimestampNs());
    } else {
      runtime_telemetry.can.steer.age_s = std::numeric_limits<double>::infinity();
    }

    runtime_telemetry.can.front_drive.valid = can_interface.HasFrontDrive();
    if (runtime_telemetry.can.front_drive.valid) {
      runtime_telemetry.can.front_drive.value = can_interface.RawFrontDrive();
      runtime_telemetry.can.front_drive.age_s = compute_age_seconds(can_interface.LastFrontDriveTimestampNs());
    } else {
      runtime_telemetry.can.front_drive.age_s = std::numeric_limits<double>::infinity();
    }

    runtime_telemetry.can.rear_drive.valid = can_interface.HasRearDrive();
    if (runtime_telemetry.can.rear_drive.valid) {
      runtime_telemetry.can.rear_drive.value = can_interface.RawRearDrive();
      runtime_telemetry.can.rear_drive.age_s = compute_age_seconds(can_interface.LastRearDriveTimestampNs());
    } else {
      runtime_telemetry.can.rear_drive.age_s = std::numeric_limits<double>::infinity();
    }

    runtime_telemetry.can.brake.valid = can_interface.HasBrake();
    if (runtime_telemetry.can.brake.valid) {
      runtime_telemetry.can.brake.value = can_interface.RawBrake();
      runtime_telemetry.can.brake.age_s = compute_age_seconds(can_interface.LastBrakeTimestampNs());
    } else {
      runtime_telemetry.can.brake.age_s = std::numeric_limits<double>::infinity();
    }

    runtime_telemetry.can.speeds.valid = can_interface.HasSpeeds();
    if (runtime_telemetry.can.speeds.valid) {
      runtime_telemetry.can.speeds.value = can_interface.RawSpeeds();
      runtime_telemetry.can.speeds.age_s = compute_age_seconds(can_interface.LastSpeedsTimestampNs());
    } else {
      runtime_telemetry.can.speeds.age_s = std::numeric_limits<double>::infinity();
    }

    runtime_telemetry.can.dynamics.valid = can_interface.HasDynamics();
    if (runtime_telemetry.can.dynamics.valid) {
      runtime_telemetry.can.dynamics.value = can_interface.RawDynamics();
      runtime_telemetry.can.dynamics.age_s = compute_age_seconds(can_interface.LastDynamicsTimestampNs());
    } else {
      runtime_telemetry.can.dynamics.age_s = std::numeric_limits<double>::infinity();
    }
    runtime_telemetry.can.vehicle_state = can_vehicle_state;
    runtime_telemetry.can.imu = can_interface.LatestImu();
    runtime_telemetry.can.gps = can_interface.LatestGps();

    runtime_telemetry.control.control_cmd = ai_control_cmd;
    runtime_telemetry.control.applied_throttle = appliedThrottle;
    runtime_telemetry.control.applied_brake = appliedBrake;
    runtime_telemetry.control.applied_steer_rad = appliedSteer;
    runtime_telemetry.control.ai_command = ai_command_sample;
    runtime_telemetry.control.ai_command_enabled = ai_command_enabled;
    runtime_telemetry.control.ai_command_applied = ai_command_applied;
    runtime_telemetry.control.has_last_command = has_last_ai_command;
    if (has_last_ai_command) {
      runtime_telemetry.control.last_command = last_ai2vcu_commands;
    }

    runtime_telemetry.mode.runtime_mode = mode;

    DrawSimulationPanel(runtime_telemetry);
    DrawControlPanel(runtime_telemetry);
    DrawCanPanel(runtime_telemetry);
    DrawLogConsole();

    steer_delay.push(now_ns, static_cast<float>(actual_steer_deg +
                                                sample_noise(sensor_cfg.steering_deg.noise_std)));
    if (auto sample = steer_delay.poll(now_ns)) {
      steer_meas_deg = *sample;
      has_steer_meas = true;
    }

    std::array<float, 4> wheel_rpm_sample{wheel_info.lf_speed, wheel_info.rf_speed,
                                          wheel_info.lb_speed, wheel_info.rb_speed};
    for (float& rpm : wheel_rpm_sample) {
      rpm = static_cast<float>(rpm + sample_noise(sensor_cfg.wheel_rpm.noise_std));
      if (!std::isfinite(rpm) || rpm < 0.0f) {
        rpm = 0.0f;
      }
    }
    wheel_delay.push(now_ns, wheel_rpm_sample);
    if (auto sample = wheel_delay.poll(now_ns)) {
      wheel_meas_rpm = *sample;
      has_wheel_meas = true;
    }

    TorqueSample torque_sample{};
    torque_sample.front_nm = static_cast<float>((pt_status.front_drive_force - pt_status.front_regen_force) *
                                               wheel_radius +
                                               sample_noise(sensor_cfg.drive_torque.front_noise_std_nm));
    torque_sample.rear_nm = static_cast<float>((pt_status.rear_drive_force - pt_status.rear_regen_force) *
                                              wheel_radius +
                                              sample_noise(sensor_cfg.drive_torque.rear_noise_std_nm));
    torque_delay.push(now_ns, torque_sample);
    if (auto sample = torque_delay.poll(now_ns)) {
      torque_meas = *sample;
      has_torque_meas = true;
    }

    fsai::sim::svcu::dbc::Ai2LogDynamics2 imu_sample{};
    imu_sample.accel_longitudinal_mps2 = static_cast<float>(
        vehicle_state.acceleration.x() + sample_noise(sensor_cfg.imu.accel_longitudinal_std));
    imu_sample.accel_lateral_mps2 = static_cast<float>(
        vehicle_state.acceleration.y() + sample_noise(sensor_cfg.imu.accel_lateral_std));
    imu_sample.yaw_rate_degps = static_cast<float>(
        vehicle_state.rotation.z() * fsai::sim::svcu::dbc::kRadToDeg +
        sample_noise(sensor_cfg.imu.yaw_rate_std_degps));
    imu_delay.push(now_ns, imu_sample);
    if (auto sample = imu_delay.poll(now_ns)) {
      imu_meas = *sample;
      has_imu_meas = true;
    }

    GpsSample gps_sample{};
    gps_sample.lat_deg = metersToLatitude(vehicle_state.position.y()) +
                         sample_noise(sensor_cfg.gps.lat_std_deg);
    gps_sample.lon_deg = metersToLongitude(vehicle_state.position.x()) +
                         sample_noise(sensor_cfg.gps.lon_std_deg);
    gps_sample.speed_mps = std::max(0.0, vehicle_speed_mps +
                                    sample_noise(sensor_cfg.gps.speed_std_mps));
    gps_delay.push(now_ns, gps_sample);
    if (auto sample = gps_delay.poll(now_ns)) {
      gps_meas = *sample;
      has_gps_meas = true;
    }

    if (has_gps_meas) {
      can_interface.SetSimulationGpsSample(gps_meas.lat_deg, gps_meas.lon_deg,
                                          gps_meas.speed_mps);
    } else {
      const double lat_deg = metersToLatitude(vehicle_state.position.y());
      const double lon_deg = metersToLongitude(vehicle_state.position.x());
      const double speed_mps_est =
          std::sqrt(vehicle_state.velocity.x() * vehicle_state.velocity.x() +
                    vehicle_state.velocity.y() * vehicle_state.velocity.y());
      can_interface.SetSimulationGpsSample(lat_deg, lon_deg, speed_mps_est);
    }

    can_interface.Poll(now_ns);

    fsai::sim::svcu::TelemetryPacket telemetry{};
    telemetry.t_ns = now_ns;
    telemetry.steer_angle_rad = has_steer_meas
                                    ? static_cast<float>(steer_meas_deg *
                                                        fsai::sim::svcu::dbc::kDegToRad)
                                    : appliedSteer;
    telemetry.front_axle_torque_nm = has_torque_meas
                                         ? torque_meas.front_nm
                                         : static_cast<float>(front_net_force * wheel_radius);
    telemetry.rear_axle_torque_nm = has_torque_meas
                                        ? torque_meas.rear_nm
                                        : static_cast<float>(rear_net_force * wheel_radius);
    telemetry.wheel_speed_rpm[0] = has_wheel_meas ? wheel_meas_rpm[0] : wheel_info.lf_speed;
    telemetry.wheel_speed_rpm[1] = has_wheel_meas ? wheel_meas_rpm[1] : wheel_info.rf_speed;
    telemetry.wheel_speed_rpm[2] = has_wheel_meas ? wheel_meas_rpm[2] : wheel_info.lb_speed;
    telemetry.wheel_speed_rpm[3] = has_wheel_meas ? wheel_meas_rpm[3] : wheel_info.rb_speed;
    telemetry.brake_pressure_front_bar =
        static_cast<float>(std::max(0.0, brake_status.front_force) / 1000.0);
    telemetry.brake_pressure_rear_bar =
        static_cast<float>(std::max(0.0, brake_status.rear_force) / 1000.0);
    telemetry.imu_ax_mps2 = has_imu_meas ? imu_meas.accel_longitudinal_mps2
                                         : static_cast<float>(vehicle_state.acceleration.x());
    telemetry.imu_ay_mps2 = has_imu_meas ? imu_meas.accel_lateral_mps2
                                         : static_cast<float>(vehicle_state.acceleration.y());
    telemetry.imu_yaw_rate_rps = has_imu_meas
                                     ? static_cast<float>(imu_meas.yaw_rate_degps *
                                                         fsai::sim::svcu::dbc::kDegToRad)
                                     : static_cast<float>(vehicle_state.rotation.z());
    const bool use_measured_gps = has_gps_meas;
    telemetry.gps_lat_deg = use_measured_gps
                                ? static_cast<float>(gps_meas.lat_deg)
                                : static_cast<float>(metersToLatitude(vehicle_state.position.y()));
    telemetry.gps_lon_deg = use_measured_gps
                                ? static_cast<float>(gps_meas.lon_deg)
                                : static_cast<float>(metersToLongitude(vehicle_state.position.x()));
    telemetry.gps_speed_mps = use_measured_gps
                                  ? static_cast<float>(gps_meas.speed_mps)
                                  : static_cast<float>(
                                        std::sqrt(vehicle_state.velocity.x() *
                                                  vehicle_state.velocity.x() +
                                                  vehicle_state.velocity.y() *
                                                      vehicle_state.velocity.y()));
    constexpr uint8_t kTelemetryFlagHvilClosed = 0x01;
    constexpr uint8_t kTelemetryFlagEbsReleased = 0x02;
    telemetry.status_flags = 0u;
    if (ai_command_enabled) {
      telemetry.status_flags |= kTelemetryFlagHvilClosed;
    }
    if (ai_command_applied) {
      telemetry.status_flags |= kTelemetryFlagEbsReleased;
    }
    telemetry_tx.send(&telemetry, sizeof(telemetry));

    logger.logState(sim_time_s, world.vehicleState());
    logger.logControl(sim_time_s, world.throttleInput, world.steeringAngle);

    if (stereo_source) {
      const auto& transform = world.vehicleTransform();
      stereo_source->setBodyPose(transform.position.x, transform.position.y,
                                 transform.position.z, transform.yaw);

      cone_positions.clear();
      const auto& left_cones = world.leftConePositions();
      const auto& right_cones = world.rightConePositions();
      cone_positions.reserve(left_cones.size() + right_cones.size());
      for (const auto& cone : left_cones) {
        cone_positions.push_back({cone.x, cone.y, cone.z});
      }
      for (const auto& cone : right_cones) {
        cone_positions.push_back({cone.x, cone.y, cone.z});
      }
      stereo_source->setCones(cone_positions);
      const FsaiStereoFrame& frame = stereo_source->capture(now_ns);
      stereo_display.present(frame);
    }

    DrawWorldScene(&graphics, world, runtime_telemetry);
    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), graphics.renderer);
    Graphics_Present(&graphics);

    SDL_Delay(static_cast<Uint32>(step_seconds * 1000.0));

    frame_counter++;
  }

  svcu_keep_running.store(false, std::memory_order_relaxed);
  if (svcu_thread.joinable()) {
    svcu_thread.join();
  }
  if (svcu_stats.has_value() && !svcu_stats->ok) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "SVCU runner exited with errors (command_tx=%llu, telemetry_rx=%llu)",
                         static_cast<unsigned long long>(svcu_stats->command_packets_sent),
                         static_cast<unsigned long long>(svcu_stats->telemetry_packets_processed));
  }

  shutdown_imgui();
  Graphics_Cleanup(&graphics);
  fsai::sim::log::LogInfoToStdout(
      "Simulation complete. Car state log saved to CarStateLog.csv");
  return EXIT_SUCCESS;
}
