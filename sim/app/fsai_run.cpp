#include <algorithm>
#include <array>
#include <chrono>
#include <numbers>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include <SDL.h>
#include <yaml-cpp/yaml.h>
#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"

#include "Graphics.h"
#include "budget.h"
#include "fsai_clock.h"
#include "csv_logger.hpp"
#include "logging.hpp"
#include "gui_world_adapter.hpp"
#include "provider_registry.hpp"
#include "stereo_display.hpp"
#include "sim_stereo_source.hpp"
#include "edge_preview.hpp"
#include "vision/frame_ring_buffer.hpp"
#include "vision/vision_node.hpp"
#include "vision/detection_preview.hpp"
#include "vision/shared_ring_buffer.hpp"
#include "io_bus.hpp"
#include "vision/detection_buffer_registry.hpp"
#include "types.h"
#include "World.hpp"
#include "VehicleDynamics.hpp"
#include "sim/cone_constants.hpp"
#include "sim/mission/MissionDefinition.hpp"
#include "sim/mission/TrackCsvLoader.hpp"
#include "adsdv_dbc.hpp"
#include "link.hpp"
#include "can_link.hpp"
#include "ai2vcu_adapter.hpp"
#include "can_iface.hpp"
#include "runtime_telemetry.hpp"
#include "centerline.hpp"
#include "human/IUserInput.hpp"

namespace {
constexpr std::size_t kDefaultMissionIndex = 2;

struct MissionOption {
  fsai::sim::MissionDescriptor descriptor;
  std::array<const char*, 4> tokens{};
};

const std::array<MissionOption, 4> kMissionOptions = {
    MissionOption{fsai::sim::MissionDescriptor{fsai::sim::MissionType::kAcceleration,
                                              "Acceleration", "acceleration"},
                  {"acceleration", "accel", "drag", "straight"}},
    MissionOption{fsai::sim::MissionDescriptor{fsai::sim::MissionType::kSkidpad,
                                              "Skidpad", "skidpad"},
                  {"skidpad", "skid", "circle", "pad"}},
    MissionOption{fsai::sim::MissionDescriptor{fsai::sim::MissionType::kAutocross,
                                              "Autocross", "autocross"},
                  {"autocross", "auto", "cross", "ax"}},
    MissionOption{fsai::sim::MissionDescriptor{fsai::sim::MissionType::kTrackdrive,
                                              "Trackdrive", "trackdrive"},
                  {"trackdrive", "track", "endurance", "td"}},
};

std::string ToLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return text;
}

std::string Trim(std::string_view text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string_view::npos) {
    return std::string();
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return std::string(text.substr(begin, end - begin + 1));
}

const MissionOption* LookupMissionByToken(std::string_view token) {
  const std::string normalized = ToLower(std::string(token));
  for (const auto& option : kMissionOptions) {
    if (normalized == option.descriptor.short_name) {
      return &option;
    }
    for (const char* alias : option.tokens) {
      if (alias != nullptr && normalized == alias) {
        return &option;
      }
    }
  }
  return nullptr;
}

std::optional<fsai::sim::MissionDescriptor> ParseMissionDescriptor(std::string_view text) {
  const std::string trimmed = Trim(text);
  if (trimmed.empty()) {
    return std::nullopt;
  }

  char* end = nullptr;
  const long numeric = std::strtol(trimmed.c_str(), &end, 10);
  if (end && *end == '\0' && end != trimmed.c_str()) {
    if (numeric >= 1 && numeric <= static_cast<long>(kMissionOptions.size())) {
      return kMissionOptions[static_cast<std::size_t>(numeric - 1)].descriptor;
    }
    return std::nullopt;
  }

  if (const auto* option = LookupMissionByToken(trimmed)) {
    return option->descriptor;
  }

  return std::nullopt;
}

bool StdinIsInteractive() {
#if defined(_WIN32)
  return _isatty(_fileno(stdin)) != 0;
#else
  return ::isatty(fileno(stdin)) != 0;
#endif
}

fsai::sim::MissionDescriptor ResolveMissionSelection(
    human::IUserInput& user_input) {
  if (const auto override = user_input.ConsumeMissionOverride()) {
    return *override;
  }

  const MissionOption& default_option = kMissionOptions[kDefaultMissionIndex];
  if (!StdinIsInteractive()) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                         "No mission specified; defaulting to %s",
                         default_option.descriptor.name.c_str());
    return default_option.descriptor;
  }

  std::cout << "Select a mission to simulate:" << std::endl;
  for (std::size_t i = 0; i < kMissionOptions.size(); ++i) {
    std::cout << "  " << (i + 1) << ") "
              << kMissionOptions[i].descriptor.name << std::endl;
  }
  std::cout << "Enter choice [" << (kDefaultMissionIndex + 1) << ": "
            << default_option.descriptor.name << "]: " << std::flush;

  std::string line;
  while (true) {
    if (!std::getline(std::cin, line)) {
      std::cout << std::endl
                << "No input detected; defaulting to "
                << default_option.descriptor.name << std::endl;
      return default_option.descriptor;
    }

    const auto parsed = ParseMissionDescriptor(line);
    const std::string trimmed = Trim(line);
    if (!parsed.has_value()) {
      if (trimmed.empty()) {
        std::cout << default_option.descriptor.name << " selected (default)."
                  << std::endl;
        return default_option.descriptor;
      }
      std::cout << "Unrecognized mission '" << trimmed
                << "'. Please enter a number or mission name ["
                << (kDefaultMissionIndex + 1) << ": "
                << default_option.descriptor.name << "]: " << std::flush;
      continue;
    }
    return *parsed;
  }
}

#ifndef FSAI_PROJECT_ROOT
#define FSAI_PROJECT_ROOT "."
#endif
 
std::filesystem::path MakeProjectRelativePath(const std::filesystem::path& path) {
  if (path.is_absolute()) {
    return path;
  }

  return std::filesystem::path(FSAI_PROJECT_ROOT) / path;
}
 
std::filesystem::path MakeProjectRelativePath(const std::string& path) {
  return MakeProjectRelativePath(std::filesystem::path(path));
}

fsai::sim::MissionDefinition BuildMissionDefinition(
    const fsai::sim::MissionDescriptor& descriptor) {
  fsai::sim::MissionDefinition definition;
  definition.descriptor = descriptor;

  switch (descriptor.type) {
    case fsai::sim::MissionType::kAcceleration: {
      const std::filesystem::path csv_path = MakeProjectRelativePath(std::filesystem::path("configs/tracks/acceleration.csv"));
      definition.track =
          fsai::sim::TrackData::FromTrackResult(fsai::sim::LoadTrackFromCsv(csv_path));
      definition.targetLaps = 1;
      definition.allowRegeneration = false;
      definition.trackSource = fsai::sim::TrackSource::kCsv;
      break;
    }
    case fsai::sim::MissionType::kSkidpad: {
      const std::filesystem::path csv_path = MakeProjectRelativePath(std::filesystem::path("configs/tracks/skidpad.csv"));
      definition.track =
          fsai::sim::TrackData::FromTrackResult(fsai::sim::LoadTrackFromCsv(csv_path));
      definition.targetLaps = 4;
      definition.allowRegeneration = false;
      definition.trackSource = fsai::sim::TrackSource::kCsv;
      break;
    }
    case fsai::sim::MissionType::kAutocross: {
      definition.targetLaps = 1;
      definition.allowRegeneration = true;
      definition.trackSource = fsai::sim::TrackSource::kRandom;
      break;
    }
    case fsai::sim::MissionType::kTrackdrive: {
      definition.targetLaps = 10;
      definition.allowRegeneration = true;
      definition.trackSource = fsai::sim::TrackSource::kRandom;
      break;
    }
  }

  return definition;
}


constexpr double kDefaultDt = 0.01;
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr int kReportIntervalFrames = 120;
constexpr float kConeDisplayScale = 12.0f;
constexpr uint16_t kDefaultCommandPort = fsai::sim::svcu::kDefaultCommandPort;
constexpr uint16_t kDefaultTelemetryPort = fsai::sim::svcu::kDefaultTelemetryPort;
constexpr double kCommandStaleSeconds = 0.1;
constexpr double kAckLagWarningSeconds = 0.25;
constexpr double kBaseLatitudeDeg = 37.4275;
constexpr double kBaseLongitudeDeg = -122.1697;
constexpr double kMetersPerDegreeLat = 111111.0;
constexpr const char* kDefaultSensorConfig = "configs/sim/sensors.yaml";


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

ImVec4 Rgba255(int r, int g, int b, int a = 255) {
  const float scale = 1.0f / 255.0f;
  return ImVec4(static_cast<float>(r) * scale, static_cast<float>(g) * scale,
                static_cast<float>(b) * scale, static_cast<float>(a) * scale);
}

const char* MissionTypeLabel(fsai::sim::MissionType type) {
  switch (type) {
    case fsai::sim::MissionType::kAcceleration:
      return "Acceleration";
    case fsai::sim::MissionType::kSkidpad:
      return "Skidpad";
    case fsai::sim::MissionType::kAutocross:
      return "Autocross";
    case fsai::sim::MissionType::kTrackdrive:
      return "Trackdrive";
  }
  return "Unknown";
}

const char* MissionSegmentTypeLabel(fsai::sim::MissionSegmentType type) {
  switch (type) {
    case fsai::sim::MissionSegmentType::kWarmup:
      return "Warmup Laps";
    case fsai::sim::MissionSegmentType::kTimed:
      return "Timed Laps";
    case fsai::sim::MissionSegmentType::kExit:
      return "Exit / Cooldown";
  }
  return "Unknown";
}

std::string MissionRunStatusLabel(fsai::sim::MissionRunStatus status, bool stop_commanded) {
  switch (status) {
    case fsai::sim::MissionRunStatus::kRunning:
      if (stop_commanded) {
        return "Stop Commanded";
      }
      return "Running";
    case fsai::sim::MissionRunStatus::kCompleted:
      return stop_commanded ? "Completed (Stop Commanded)" : "Completed";
  }
  return "Unknown";
}

ImVec4 MissionTypeColor(fsai::sim::MissionType type) {
  switch (type) {
    case fsai::sim::MissionType::kAcceleration:
      return Rgba255(255, 171, 64);
    case fsai::sim::MissionType::kSkidpad:
      return Rgba255(25, 188, 157);
    case fsai::sim::MissionType::kAutocross:
      return Rgba255(66, 165, 245);
    case fsai::sim::MissionType::kTrackdrive:
      return Rgba255(171, 71, 188);
  }
  return Rgba255(158, 158, 158);
}

ImVec4 MissionStatusColor(fsai::sim::MissionRunStatus status, bool stop_commanded) {
  switch (status) {
    case fsai::sim::MissionRunStatus::kRunning:
      if (stop_commanded) {
        return Rgba255(239, 108, 0);
      }
      return Rgba255(255, 213, 79);
    case fsai::sim::MissionRunStatus::kCompleted:
      return stop_commanded ? Rgba255(102, 187, 106) : Rgba255(76, 175, 80);
  }
  return Rgba255(229, 57, 53);
}

struct MissionStatusDisplay {
  std::string label;
  ImVec4 color{Rgba255(189, 189, 189)};
  bool finished{false};
};

bool CanStatusHasFault(const fsai::sim::svcu::dbc::Vcu2AiStatus& status) {
  return status.fault || status.mission_status_fault || status.ai_estop_request ||
         status.hvil_open_fault || status.hvil_short_fault || status.ebs_fault ||
         status.offboard_charger_fault || status.charge_procedure_fault ||
         status.autonomous_braking_fault || status.bms_fault ||
         status.brake_plausibility_fault;
}

MissionStatusDisplay FormatCanMissionStatus(
    const fsai::sim::svcu::dbc::Vcu2AiStatus& status) {
  MissionStatusDisplay display;
  display.finished = status.mission_complete ||
                     status.mission_status == fsai::sim::svcu::dbc::MissionStatus::kFinished;

  switch (status.mission_status) {
    case fsai::sim::svcu::dbc::MissionStatus::kRunning:
      display.label = MissionRunStatusLabel(fsai::sim::MissionRunStatus::kRunning, false);
      break;
    case fsai::sim::svcu::dbc::MissionStatus::kFinished:
      display.label = MissionRunStatusLabel(fsai::sim::MissionRunStatus::kCompleted, false);
      break;
    case fsai::sim::svcu::dbc::MissionStatus::kSelected:
      display.label = "Selected";
      break;
    case fsai::sim::svcu::dbc::MissionStatus::kNotSelected:
    default:
      display.label = "Not Selected";
      break;
  }

  if (CanStatusHasFault(status)) {
    display.color = Rgba255(229, 57, 53);
  } else if (display.finished) {
    display.color = MissionStatusColor(fsai::sim::MissionRunStatus::kCompleted, false);
  } else if (status.mission_status == fsai::sim::svcu::dbc::MissionStatus::kRunning) {
    display.color = MissionStatusColor(fsai::sim::MissionRunStatus::kRunning, false);
  }

  return display;
}

void DrawWarningBadge(std::string_view message) {
  const ImVec4 badge_color = Rgba255(255, 193, 7);
  ImGui::SameLine();
  std::string label = "⚠";
  if (!message.empty()) {
    label.append(" ");
    label.append(message);
  }
  ImGui::PushStyleColor(ImGuiCol_Text, badge_color);
  ImGui::TextUnformatted(label.c_str());
  ImGui::PopStyleColor();
}

ImVec4 MissionSegmentTypeColor(fsai::sim::MissionSegmentType type) {
  switch (type) {
    case fsai::sim::MissionSegmentType::kWarmup:
      return Rgba255(129, 212, 250);
    case fsai::sim::MissionSegmentType::kTimed:
      return Rgba255(129, 199, 132);
    case fsai::sim::MissionSegmentType::kExit:
      return Rgba255(206, 147, 216);
  }
  return Rgba255(158, 158, 158);
}

ImVec4 MissionSegmentStateColor(fsai::sim::MissionSegmentType type, bool active, bool complete) {
  if (complete) {
    return Rgba255(67, 160, 71);
  }
  if (active) {
    return MissionSegmentTypeColor(type);
  }
  return Rgba255(189, 189, 189);
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

void DrawMissionPanel(const fsai::sim::app::RuntimeTelemetry& telemetry) {
  const auto& mission = telemetry.mission;
  ImGui::Begin("Mission");

  const std::string& mission_name = mission.mission_name.empty()
                                        ? mission.mission_short_name
                                        : mission.mission_name;
  if (!mission_name.empty()) {
    ImGui::Text("%s", mission_name.c_str());
  } else {
    ImGui::TextUnformatted("Mission");
  }

  const char* mission_type_label = MissionTypeLabel(mission.mission_type);
  ImGui::TextColored(MissionTypeColor(mission.mission_type), "%s Mission",
                     mission_type_label);

  const std::string status_label = MissionRunStatusLabel(mission.status, mission.stop_commanded);
  const ImVec4 status_color = MissionStatusColor(mission.status, mission.stop_commanded);
  ImGui::TextColored(status_color, "%s", status_label.c_str());

  ImGui::Spacing();
  ImGui::Text("Mission time: %.1f s", mission.mission_time_s);

  const double total_target_laps = static_cast<double>(mission.target_laps);
  const double total_completed_laps = static_cast<double>(mission.completed_laps);
  float laps_ratio = 0.0f;
  if (total_target_laps > 0.0) {
    laps_ratio = static_cast<float>(std::clamp(total_completed_laps / total_target_laps, 0.0, 1.0));
  } else if (mission.status == fsai::sim::MissionRunStatus::kCompleted) {
    laps_ratio = 1.0f;
  }
  char lap_progress_label[64];
  std::snprintf(lap_progress_label, sizeof(lap_progress_label), "%d / %zu laps",
                mission.completed_laps, mission.target_laps);
  ImGui::ProgressBar(laps_ratio, ImVec2(-FLT_MIN, 0.0f), lap_progress_label);

  if (!mission.segments.empty()) {
    ImGui::Spacing();
    if (ImGui::BeginTable("mission_segments", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("Segment");
      ImGui::TableSetupColumn("Status");
      ImGui::TableSetupColumn("Progress");
      ImGui::TableSetupColumn("Elapsed");
      ImGui::TableHeadersRow();

      for (std::size_t i = 0; i < mission.segments.size(); ++i) {
        const auto& segment = mission.segments[i];
        const bool complete = segment.target_laps > 0 &&
                              segment.completed_laps >= segment.target_laps;
        const bool active = mission.active_segment_index.has_value() &&
                            mission.active_segment_index == i;
        const ImVec4 segment_color =
            MissionSegmentStateColor(segment.type, active, complete);

        ImGui::TableNextRow();
        if (active) {
          ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                 ImGui::GetColorU32(ImVec4(0.15f, 0.25f, 0.45f, 0.25f)));
        }

        ImGui::TableSetColumnIndex(0);
        ImGui::PushStyleColor(ImGuiCol_Text, MissionSegmentTypeColor(segment.type));
        ImGui::Text("%s", MissionSegmentTypeLabel(segment.type));
        ImGui::PopStyleColor();

        ImGui::TableSetColumnIndex(1);
        const char* state_label = complete ? "Complete" : (active ? "In progress" : "Queued");
        ImGui::PushStyleColor(ImGuiCol_Text, segment_color);
        ImGui::TextUnformatted(state_label);
        ImGui::PopStyleColor();

        ImGui::TableSetColumnIndex(2);
        float segment_ratio = 0.0f;
        if (segment.target_laps > 0) {
          segment_ratio =
              static_cast<float>(std::clamp(static_cast<double>(segment.completed_laps) /
                                                 static_cast<double>(segment.target_laps),
                                             0.0, 1.0));
        } else if (complete) {
          segment_ratio = 1.0f;
        }
        char segment_label[48];
        std::snprintf(segment_label, sizeof(segment_label), "%zu / %zu",
                      segment.completed_laps, segment.target_laps);
        ImGui::ProgressBar(segment_ratio, ImVec2(-FLT_MIN, 0.0f), segment_label);

        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.1f s", segment.elapsed_time_s);
      }
      ImGui::EndTable();
    }
  } else {
    ImGui::Spacing();
    const bool has_target = mission.segment_target_laps > 0;
    float segment_ratio = 0.0f;
    if (has_target) {
      segment_ratio = static_cast<float>(std::clamp(
          static_cast<double>(mission.segment_completed_laps) /
              static_cast<double>(mission.segment_target_laps),
          0.0, 1.0));
    } else if (mission.status == fsai::sim::MissionRunStatus::kCompleted) {
      segment_ratio = 1.0f;
    }
    char segment_label[64];
    if (has_target) {
      std::snprintf(segment_label, sizeof(segment_label), "%s: %zu / %zu",
                    MissionSegmentTypeLabel(mission.segment), mission.segment_completed_laps,
                    mission.segment_target_laps);
    } else {
      std::snprintf(segment_label, sizeof(segment_label), "%s", MissionSegmentTypeLabel(mission.segment));
    }
    ImGui::ProgressBar(segment_ratio, ImVec2(-FLT_MIN, 0.0f), segment_label);
  }

  if (mission.active_segment_index.has_value()) {
    const std::size_t index = *mission.active_segment_index;
    if (index < mission.segments.size()) {
      const auto& active = mission.segments[index];
      ImGui::Spacing();
      ImGui::TextColored(MissionSegmentTypeColor(active.type), "Active segment progress");
      float active_ratio = 0.0f;
      const bool active_has_target = active.target_laps > 0;
      if (active_has_target) {
        active_ratio = static_cast<float>(std::clamp(
            static_cast<double>(active.completed_laps) /
                static_cast<double>(active.target_laps),
            0.0, 1.0));
      } else if (mission.status == fsai::sim::MissionRunStatus::kCompleted) {
        active_ratio = 1.0f;
      }
      char active_label[48];
      if (active_has_target) {
        std::snprintf(active_label, sizeof(active_label), "%zu / %zu laps",
                      active.completed_laps, active.target_laps);
      } else {
        std::snprintf(active_label, sizeof(active_label), "%s", MissionSegmentTypeLabel(active.type));
      }
      ImGui::ProgressBar(active_ratio, ImVec2(-FLT_MIN, 0.0f), active_label);
    }
  }

  if (mission.straight_progress_m > 0.0) {
    static std::string last_mission_id;
    static double straight_span_m = 1.0;
    const std::string& mission_id = mission.mission_short_name.empty()
                                        ? mission_name
                                        : mission.mission_short_name;
    if (mission_id != last_mission_id || (mission.mission_time_s < 0.1 &&
                                          mission.completed_laps == 0)) {
      last_mission_id = mission_id;
      straight_span_m = std::max(1.0, mission.straight_progress_m);
    }
    straight_span_m = std::max(straight_span_m, std::max(1.0, mission.straight_progress_m));
    float straight_ratio = static_cast<float>(std::clamp(
        mission.straight_progress_m / straight_span_m, 0.0, 1.0));
    char straight_label[64];
    std::snprintf(straight_label, sizeof(straight_label), "%.1f m", mission.straight_progress_m);
    ImGui::Spacing();
    ImGui::TextUnformatted("Straight-line progress");
    ImGui::ProgressBar(straight_ratio, ImVec2(-FLT_MIN, 0.0f), straight_label);
  } else {
    ImGui::Spacing();
    ImGui::Text("Straight-line progress: %.1f m", mission.straight_progress_m);
  }

  ImGui::End();
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
  ImGui::Text("Control mode: %s",
              telemetry.mode.use_controller ? "automatic" : "manual");
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
  const bool fallback_manual = telemetry.control.fallback_to_manual;

  ImGui::Text("Control mode: %s",
              telemetry.mode.use_controller ? "automatic" : "manual");
  ImGui::Text("AI command stream: %s", ai_enabled ? "enabled" : "disabled");

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

  if (fallback_manual) {
    ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f),
                       "Falling back to manual inputs");
  } else if (ai_applied) {
    ImGui::TextColored(ImVec4(0.2f, 0.75f, 0.2f, 1.0f), "AI command applied");
  } else {
    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.25f, 1.0f),
                       "Manual control active");
  }

  ImGui::Separator();
  ImGui::Text("Applied command");
  ImGui::Text("Throttle: %.2f | Brake: %.2f | Steer: %.1f deg",
              telemetry.control.applied_throttle,
              telemetry.control.applied_brake,
              telemetry.control.applied_steer_rad *
                  fsai::sim::svcu::dbc::kRadToDeg);
  ImGui::Text("Source: %s", ai_applied ? "AI" : "Manual");

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
      const bool has_mission_feedback =
          status.mission_id != 0 || status.mission_complete ||
          status.mission_status != fsai::sim::svcu::dbc::MissionStatus::kNotSelected;
      if (has_mission_feedback) {
        ImGui::Text("Mission ID: %u", static_cast<unsigned int>(status.mission_id));

        const MissionStatusDisplay mission_display = FormatCanMissionStatus(status);
        ImGui::TextColored(mission_display.color, "Mission status: %s",
                           mission_display.label.c_str());

        const bool simulator_completed =
            telemetry.mission.status == fsai::sim::MissionRunStatus::kCompleted;
        ImGui::TextColored(mission_display.color, "Mission complete: %s",
                           status.mission_complete ? "Yes" : "No");
        std::string discrepancy_message;
        if (simulator_completed && !mission_display.finished) {
          discrepancy_message = "Sim complete; CAN pending";
        } else if (!simulator_completed && mission_display.finished) {
          discrepancy_message = "CAN complete; sim running";
        }
        if (!discrepancy_message.empty()) {
          DrawWarningBadge(discrepancy_message);
        }
      } else {
        ImGui::Text("Mission ID: n/a");
        ImGui::TextColored(Rgba255(189, 189, 189), "Mission status: Not reported");
        ImGui::TextColored(Rgba255(189, 189, 189), "Mission complete: No");
      }

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

void DrawEdgePreviewPanel(fsai::vision::EdgePreview& preview, uint64_t now_ns) {
  ImGui::Begin("Edge Preview");

  const auto snapshot = preview.snapshot();
  if (!snapshot.texture) {
    const std::string status = preview.statusMessage();
    if (!status.empty()) {
      ImGui::TextWrapped("%s", status.c_str());
    } else {
      ImGui::TextUnformatted("Waiting for edge frames...");
    }
    ImGui::End();
    return;
  }

  ImGui::Text("Resolution: %d x %d", snapshot.width, snapshot.height);
  double age_ms = std::numeric_limits<double>::infinity();
  if (snapshot.frame_timestamp_ns != 0 && now_ns >= snapshot.frame_timestamp_ns) {
    age_ms = static_cast<double>(now_ns - snapshot.frame_timestamp_ns) * 1e-6;
  }
  if (std::isfinite(age_ms)) {
    ImGui::Text("Frame age: %.1f ms", age_ms);
  } else {
    ImGui::Text("Frame age: n/a");
  }

  float display_width = static_cast<float>(snapshot.width);
  float display_height = static_cast<float>(snapshot.height);
  if (display_width <= 0.0f || display_height <= 0.0f) {
    display_width = 1.0f;
    display_height = 1.0f;
  }
  const float max_width = 480.0f;
  const float max_height = 360.0f;
  if (display_width > max_width) {
    const float scale = max_width / display_width;
    display_width *= scale;
    display_height *= scale;
  }
  if (display_height > max_height) {
    const float scale = max_height / display_height;
    display_width *= scale;
    display_height *= scale;
  }

  ImGui::Image(snapshot.texture.get(), ImVec2(display_width, display_height));

  ImGui::End();
}


void DrawDetectionPreviewPanel(fsai::vision::DetectionPreview& preview, uint64_t now_ns) {
  // Force position and size every frame to guarantee visibility
  // ImGui::SetNextWindowPos(ImVec2(100, 400), ImGuiCond_Always);
  // ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_Always);

  if (ImGui::Begin("Cone detection", nullptr)) {
      ImGui::Text("Model used: Yolov8 ");
      ImGui::Separator();

      // Simple diagnostic to see if the internal state is what we think
      auto snapshot = preview.snapshot();
      if (snapshot.texture) {
           ImGui::TextColored(ImVec4(0,1,0,1), "Texture READY: %dx%d",
                              snapshot.width, snapshot.height);
           // Try drawing it small to see if the texture itself crashes it
           ImGui::Image(snapshot.texture.get(), ImVec2(400,300));
      } else {
           ImGui::TextColored(ImVec4(1,0,0,1), "Texture NOT READY");
           ImGui::Text("Status: %s", preview.statusMessage().c_str());
      }
  }
  ImGui::End();
}
namespace {

void DrawConeMarker(Graphics* graphics, int center_x, int center_y,
                    float base_width_m, const SDL_Color& color) {
  if (graphics == nullptr || graphics->renderer == nullptr) {
    return;
  }

  const float base_radius_px =
      0.5f * base_width_m * K_RENDER_SCALE * kConeDisplayScale;
  const int radius_px =
      std::max(1, static_cast<int>(std::lround(base_radius_px)));

  SDL_SetRenderDrawColor(graphics->renderer, color.r, color.g, color.b,
                         color.a);
  Graphics_DrawFilledCircle(graphics, center_x, center_y, radius_px);
}

}  // namespace

void DrawWorldScene(Graphics* graphics,
                    const fsai::sim::app::GuiWorldSnapshot& world,
                    const fsai::sim::app::RuntimeTelemetry& telemetry) {
  (void)telemetry;
  fsai::time::SimulationStageTimer render_timer("renderer");
  Graphics_Clear(graphics);
  Graphics_DrawGrid(graphics, 50);

  const auto& left_cones = world.left_cones;
  const auto& right_cones = world.right_cones;
  const std::size_t gate_count =
      std::min(left_cones.size(), right_cones.size());

  if (gate_count > 0) {
    for (std::size_t i = 0; i < gate_count; ++i) {
      const bool is_current_gate = (i == 0);
      const SDL_Color color = is_current_gate
                                  ? SDL_Color{200, 0, 200, 255}
                                  : SDL_Color{120, 120, 200, 180};
      const auto& left = left_cones[i];
      const auto& right = right_cones[i];

      const float left_x = left.x * K_RENDER_SCALE + graphics->width / 2.0f;
      const float left_y = left.z * K_RENDER_SCALE + graphics->height / 2.0f;
      const float right_x = right.x * K_RENDER_SCALE + graphics->width / 2.0f;
      const float right_y = right.z * K_RENDER_SCALE + graphics->height / 2.0f;

      SDL_SetRenderDrawColor(graphics->renderer, color.r, color.g, color.b,
                             color.a);
      SDL_RenderDrawLineF(graphics->renderer, left_x, left_y, right_x,
                          right_y);

      if (is_current_gate) {
        const float thickness = std::max(1.5f, K_RENDER_SCALE * 0.15f);
        const float dx = right_x - left_x;
        const float dy = right_y - left_y;
        const float length = std::hypot(dx, dy);
        if (length > std::numeric_limits<float>::epsilon()) {
          const float nx = -dy / length;
          const float ny = dx / length;
          const float offset_x = nx * thickness * 0.5f;
          const float offset_y = ny * thickness * 0.5f;
          SDL_RenderDrawLineF(graphics->renderer, left_x + offset_x,
                              left_y + offset_y, right_x + offset_x,
                              right_y + offset_y);
          SDL_RenderDrawLineF(graphics->renderer, left_x - offset_x,
                              left_y - offset_y, right_x - offset_x,
                              right_y - offset_y);
        }
      }
    }
  } else {
    const auto& checkpoints = world.checkpoints;
    if (!checkpoints.empty()) {
      SDL_SetRenderDrawColor(graphics->renderer, 200, 0, 200, 255);
      Graphics_DrawFilledCircle(
          graphics,
          static_cast<int>(checkpoints.front().x * K_RENDER_SCALE +
                           graphics->width / 2.0f),
          static_cast<int>(checkpoints.front().z * K_RENDER_SCALE +
                           graphics->height / 2.0f),
          static_cast<int>(K_RENDER_SCALE));
    }
  }

  const auto& lookahead = world.lookahead;

  const auto& start_cones = world.start_cones;
  const SDL_Color start_color{255, 140, 0, 255};
  for (const auto& cone : start_cones) {
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
    DrawConeMarker(graphics, cone_x, cone_y,
                  fsai::sim::kLargeConeRadiusMeters * 2.0f, start_color);
  }
  // Blue 0, 102, 204, 255
  const SDL_Color left_base{255, 214, 0, 255};
  for (size_t i = 0; i < world.left_cones.size(); ++i) {
    SDL_Color color = left_base;
    if (i == 0) {
      color = SDL_Color{0, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.speed) {
      color = SDL_Color{255, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.steer) {
      color = SDL_Color{255, 0, 255, 255};
    }
    const auto& cone = world.left_cones[i];
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
    DrawConeMarker(graphics, cone_x, cone_y,
                  fsai::sim::kSmallConeRadiusMeters * 2.0f, color);
  }
  // Yellow 255, 214, 0, 255
  const SDL_Color right_base{0, 102, 204, 255};
  for (size_t i = 0; i < world.right_cones.size(); ++i) {
    SDL_Color color = right_base;
    if (i == 0) {
      color = SDL_Color{0, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.speed) {
      color = SDL_Color{255, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.steer) {
      color = SDL_Color{255, 0, 255, 255};
    }
    const auto& cone = world.right_cones[i];
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
    DrawConeMarker(graphics, cone_x, cone_y,
                  fsai::sim::kSmallConeRadiusMeters * 2.0f, color);
  }

  const auto& transform = world.vehicle_transform;
  const float car_screen_x = transform.position.x * K_RENDER_SCALE +
                             graphics->width / 2.0f;
  const float car_screen_y = transform.position.z * K_RENDER_SCALE +
                             graphics->height / 2.0f;
  const float car_radius = 2.0f * K_RENDER_SCALE;
  Graphics_DrawCar(graphics, car_screen_x, car_screen_y, car_radius,
                   transform.yaw);

    if (world.detections != nullptr) {
      for (const auto& cone : *world.detections) {
          // printf("\n\n cone conf %f \n\n", cone.conf);
          int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
          int cone_y = static_cast<int>(cone.y * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
          if (cone.side == FSAI_CONE_LEFT) {
            SDL_SetRenderDrawColor(graphics->renderer, 5, 200, 5, 255);
          } else if (cone.side == FSAI_CONE_RIGHT) {
            SDL_SetRenderDrawColor(graphics->renderer, 200, 5, 5, 255);
          } else if (cone.side == FSAI_CONE_UNKNOWN) {
            SDL_SetRenderDrawColor(graphics->renderer, 150, 150, 150, 250);
          }
          Graphics_DrawFilledCircle(graphics, cone_x, cone_y, 5);
      }
    }
  auto to_cones = [](const std::vector<Vector3>& positions, ConeType type) {
    std::vector<Cone> cones;
    cones.reserve(positions.size());
    for (const auto& pos : positions) {
      Cone cone{};
      cone.position = pos;
      cone.type = type;
      cones.push_back(cone);
    }
    return cones;
  };

  std::vector<std::pair<Vector2, Vector2>> triangulationEdges =
      getVisibleTriangulationEdges(world.vehicle_state,
                                   to_cones(world.left_cones, ConeType::Left),
                                   to_cones(world.right_cones, ConeType::Right))
          .second;
  for (auto edge: triangulationEdges) {
    Graphics_DrawSegment(graphics, edge.first.x, edge.first.y, edge.second.x, edge.second.y, 50, 0, 255);
  }
  for (auto edge: world.best_path_edges) {
    Graphics_DrawSegment(graphics, edge.first.x, edge.first.y, edge.second.x, edge.second.y, 255, 50, 50);
  }
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
  bool edge_preview_enabled{true};
  bool dection_preview_enabled{true};
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

struct BrakeSample {
  float front_pct{0.0f};
  float rear_pct{0.0f};
  float front_req_pct{0.0f};
  float rear_req_pct{0.0f};
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
    if (auto vision = root["vision"]) {
      if (auto preview = vision["edge_preview"]) {
        cfg.edge_preview_enabled = preview.as<bool>(cfg.edge_preview_enabled);
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Failed to load sensor config '%s': %s\n", path.c_str(), e.what());
  }
  return cfg;
}

struct ThreadSafeTelemetry {

    std::mutex mutex;
    Eigen::Vector2d position{0.0, 0.0};
    double yaw_rad = 0.0;

};

class LocalUserInput final : public human::IUserInput {
 public:
  void PublishEmergencyStop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stop_requested_ = true;
  }

  void PublishMissionOverride(
      const fsai::sim::MissionDescriptor& mission) override {
    std::lock_guard<std::mutex> lock(mutex_);
    mission_override_ = mission;
  }

  bool ConsumeEmergencyStop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool requested = emergency_stop_requested_;
    emergency_stop_requested_ = false;
    return requested;
  }

  std::optional<fsai::sim::MissionDescriptor> ConsumeMissionOverride() override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto value = mission_override_;
    mission_override_.reset();
    return value;
  }

 private:
  std::mutex mutex_{};
  bool emergency_stop_requested_{false};
  std::optional<fsai::sim::MissionDescriptor> mission_override_{};
};
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

  LocalUserInput user_input;
  double dt = kDefaultDt;
  std::string can_iface = fsai::sim::svcu::default_can_endpoint();
  std::string mode = "sim";
  bool can_iface_overridden = false;
  uint16_t command_port = kDefaultCommandPort;
  uint16_t telemetry_port = kDefaultTelemetryPort;
  std::string sensor_config_path = kDefaultSensorConfig;
  std::optional<bool> edge_preview_override;
  std::optional<bool> detection_preview_override;

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
    } else if (arg == "--edge-preview") {
      edge_preview_override = true;
    } else if (arg == "--no-edge-preview") {
      edge_preview_override = false;
    } else if (arg == "--detection-preview") {
      detection_preview_override = true;
    } else if (arg == "--no-detection-preview") {
      detection_preview_override = false;
    } else if (arg == "--mission" && i + 1 < argc) {
      const std::string mission_value = argv[++i];
      const auto parsed_mission = ParseMissionDescriptor(mission_value);
      if (!parsed_mission.has_value()) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                             "Unrecognized mission '%s'", mission_value.c_str());
        return EXIT_FAILURE;
      }
      user_input.PublishMissionOverride(*parsed_mission);
    } else if (arg == "--mission") {
      fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                           "--mission flag requires an argument");
      return EXIT_FAILURE;
    } else if (arg == "--help") {
      fsai::sim::log::LogfToStdout(
          fsai::sim::log::Level::kInfo,
          "Usage: fsai_run [--dt seconds] [--can-if iface|udp:port] [--cmd-port p] "
          "[--state-port p] [--sensor-config path] [--mission name] "
          "[--edge-preview|--no-edge-preview] [--detection-preview|--no-detection_preview]");
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
  can_iface = fsai::sim::svcu::canonicalize_can_endpoint(can_iface);
  const bool can_is_udp = fsai::sim::svcu::is_udp_endpoint(can_iface);
  fsai::sim::log::Logf(
      fsai::sim::log::Level::kInfo,
      "Using dt = %.4f seconds, mode=%s, CAN %s (%s), cmd-port %u, state-port %u",
      dt, mode.c_str(), can_iface.c_str(), can_is_udp ? "udp" : "socketcan",
      command_port, telemetry_port);

  const fsai::sim::MissionDescriptor mission =
      ResolveMissionSelection(user_input);
  fsai::sim::log::Logf(fsai::sim::log::Level::kInfo, "Mission: %s",
                       mission.name.c_str());

  const fsai::sim::MissionDefinition base_mission_definition =
      BuildMissionDefinition(mission);
  const auto world_config_path =
      MakeProjectRelativePath(std::filesystem::path("configs/sim/world.yaml"));
  const WorldConfig world_config =
      LoadWorldConfig(world_config_path.string(), base_mission_definition);
  const fsai::sim::MissionDefinition& mission_definition = world_config.mission;
  const char* track_source =
      mission_definition.trackSource == fsai::sim::TrackSource::kCsv ? "CSV" : "Random";
  fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                       "Track source: %s, target laps: %zu, regeneration %s",
                       track_source, mission_definition.targetLaps,
                       mission_definition.allowRegeneration ? "enabled" : "disabled");

  SensorNoiseConfig sensor_cfg = LoadSensorNoiseConfig(MakeProjectRelativePath(sensor_config_path).string());
  bool edge_preview_enabled = sensor_cfg.edge_preview_enabled;
  if (edge_preview_override.has_value()) {
    edge_preview_enabled = *edge_preview_override;
  }
  bool detection_preview_enabled = true; // Enable by default
  if (detection_preview_override.has_value()) {
    detection_preview_enabled = *detection_preview_override; // Use the override
  }
  fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                       "Detection preview %s",
                       detection_preview_enabled ? "enabled" : "disabled");
  fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                       "Edge-detection preview %s",
                       edge_preview_enabled ? "enabled" : "disabled");
  std::mt19937 noise_rng(sensor_cfg.seed);
  auto sample_noise = [&noise_rng](double stddev) {
    if (stddev <= 0.0) {
      return 0.0;
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(noise_rng);
  };

  auto io_bus = std::make_shared<fsai::io::InProcessIoBus>();
  fsai::io::TelemetryNoiseConfig bus_noise{};
  constexpr double kDegToRad = std::numbers::pi / 180.0;
  bus_noise.steer_rad_stddev = sensor_cfg.steering_deg.noise_std * kDegToRad;
  bus_noise.axle_torque_stddev =
      std::max(sensor_cfg.drive_torque.front_noise_std_nm,
               sensor_cfg.drive_torque.rear_noise_std_nm);
  bus_noise.wheel_rpm_stddev = sensor_cfg.wheel_rpm.noise_std;
  bus_noise.brake_bar_stddev = sensor_cfg.brake_pct.noise_std * 0.01;
  bus_noise.imu_stddev = sensor_cfg.imu.accel_longitudinal_std;
  bus_noise.gps_speed_stddev = sensor_cfg.gps.speed_std_mps;
  io_bus->set_noise_config(bus_noise);

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
  SensorDelayLine<BrakeSample> brake_delay;
  SensorDelayLine<float> speed_delay;
  SensorDelayLine<fsai::sim::svcu::dbc::Ai2LogDynamics2> imu_delay;
  SensorDelayLine<GpsSample> gps_delay;

  steer_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.steering_deg.latency_s));
  wheel_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.wheel_rpm.latency_s));
  torque_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.drive_torque.latency_s));
  brake_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.brake_pct.latency_s));
  speed_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.speed_kph.latency_s));
  imu_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.imu.latency_s));
  gps_delay.set_latency(fsai_clock_from_seconds(sensor_cfg.gps.latency_s));

  float steer_meas_deg = 0.0f;
  bool has_steer_meas = false;
  std::array<float, 4> wheel_meas_rpm{0.0f, 0.0f, 0.0f, 0.0f};
  bool has_wheel_meas = false;
  TorqueSample torque_meas{};
  bool has_torque_meas = false;
  BrakeSample brake_meas{};
  bool has_brake_meas = false;
  float speed_meas_kph = 0.0f;
  bool has_speed_meas = false;
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

  const auto vehicle_config_path = MakeProjectRelativePath(std::filesystem::path("configs/vehicle/configDry.yaml"));
  const VehicleParam vehicle_param = VehicleParam::loadFromFile(vehicle_config_path.c_str());
  VehicleDynamics vehicle_dynamics(vehicle_param);
  World world;
  bool mission_finished = false;
  const auto describe_reset_reason = [](fsai::sim::WorldRuntime::ResetReason reason)
      -> const char* {
    switch (reason) {
      case fsai::sim::WorldRuntime::ResetReason::kConeCollision:
        return "cone collision";
      case fsai::sim::WorldRuntime::ResetReason::kBoundaryCollision:
        return "boundary collision";
      case fsai::sim::WorldRuntime::ResetReason::kTrackRegeneration:
        return "track regeneration";
      case fsai::sim::WorldRuntime::ResetReason::kExternal:
        return "external reset";
      case fsai::sim::WorldRuntime::ResetReason::kUnknown:
      default:
        return "unknown";
    }
  };
  world.runtime_controller().AddResetListener(
      [describe_reset_reason](const fsai::sim::WorldRuntime::ResetEvent& evt) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                             "World reset requested (%s)",
                             describe_reset_reason(evt.reason));
      });
  world.runtime_controller().AddMissionCompleteListener(
      [&mission_finished](const fsai::sim::MissionRuntimeState& state) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                             "Mission '%s' complete after %.2f s (%zu/%zu laps)",
                             state.mission().descriptor.name.c_str(),
                             state.mission_time_seconds(),
                             state.completed_laps(), state.target_laps());
        mission_finished = true;
      });
  WorldVehicleContext world_vehicle_ctx{};
  world_vehicle_ctx.dynamics = &vehicle_dynamics;
  world_vehicle_ctx.reset_handler =
      [&world, &vehicle_dynamics](const WorldVehicleSpawn& spawn) {
        vehicle_dynamics.setState(spawn.state, spawn.transform);
        world.acknowledgeVehicleReset(spawn.transform);
      };
  world.init(world_vehicle_ctx, world_config);

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
  std::shared_ptr<fsai::vision::FrameRingBuffer> stereo_frame_buffer;
  //vision_node
  std::shared_ptr<fsai::vision::VisionNode> vision_node;
  if (stereo_factory) {
    stereo_source = stereo_factory();
    if (stereo_source) {
      constexpr std::size_t kFrameRingCapacity = 4;
      stereo_frame_buffer = fsai::vision::makeFrameRingBuffer(kFrameRingCapacity);
      fsai::vision::setActiveFrameRingBuffer(stereo_frame_buffer);
    }
  }
  if (!stereo_frame_buffer) {
    fsai::vision::setActiveFrameRingBuffer(nullptr);
  }
  std::unique_ptr<fsai::sim::integration::StereoDisplay> stereo_display;
  if (edge_preview_enabled) {
    stereo_display = std::make_unique<fsai::sim::integration::StereoDisplay>();
  }

  fsai::vision::EdgePreview edge_preview;
  if (edge_preview_enabled && !stereo_frame_buffer) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "Edge preview disabled: stereo source unavailable");
    edge_preview_enabled = false;
  } else if (edge_preview_enabled && stereo_frame_buffer) {
    std::string preview_error;
    if (!edge_preview.start(graphics.renderer, stereo_frame_buffer, preview_error)) {
      if (preview_error.empty()) {
        preview_error = "edge worker initialization failed";
      }
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Edge preview disabled: %s", preview_error.c_str());
      edge_preview_enabled = false;
    }
  }

  io_bus->set_stereo_sink([stereo_frame_buffer, &stereo_display](const FsaiStereoFrame& frame) {
    if (stereo_frame_buffer) {
      while (!stereo_frame_buffer->tryPush(frame)) {
        if (!stereo_frame_buffer->tryPop().has_value()) {
          break;
        }
      }
    }
    if (stereo_display) {
      stereo_display->present(frame);
    }
  });


  fsai::sim::log::Logf(fsai::sim::log::Level::kInfo, "Starting VisionNode...");
  ThreadSafeTelemetry shared_telemetry;
  try {
    vision_node = std::make_shared<fsai::vision::VisionNode>();
    vision_node->setPoseProvider([&shared_telemetry]() {
      std::lock_guard<std::mutex> lock(shared_telemetry.mutex);
      return std::make_pair(shared_telemetry.position, shared_telemetry.yaw_rad);
    });
    vision_node->start();
    fsai::sim::log::Logf(fsai::sim::log::Level::kInfo, "VisionNode started successfully.");
  } catch (const std::exception& e) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                         "Failed to start VisionNode: %s", e.what());
    return EXIT_FAILURE;
  }
  // --------------------------------------------------------
  fsai::vision::DetectionPreview detection_preview;
  if (detection_preview_enabled) {
    if (!vision_node) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "Detection preview disabled: vision node unavailable");
      detection_preview_enabled = false;
    } else {
      // If we are here, we are good to start
      fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                            "Attempting to start DetectionPreview...");
      std::string preview_error;
      if (!detection_preview.start(graphics.renderer, vision_node, preview_error)) {
        if (preview_error.empty()) {
          preview_error = "start() returned false";
        }
        fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Detection preview disabled: %s", preview_error.c_str());
        detection_preview_enabled = false;
      } else {
        fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                            "DetectionPreview started successfully!");
      }
    }
  }
  std::vector<fsai::io::camera::sim_stereo::SimConeInstance> cone_positions;
  bool running = true;
  size_t frame_counter = 0;

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
      const std::string fallback = fsai::sim::svcu::default_can_endpoint();
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
  io_bus->set_telemetry_sink(
      [&telemetry_tx](const fsai::sim::svcu::TelemetryPacket& pkt) {
        telemetry_tx.send(&pkt, sizeof(pkt));
      });

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
  adapter_cfg.mission_descriptor = mission_definition.descriptor;
  fsai::control::runtime::Ai2VcuAdapter ai2vcu_adapter(adapter_cfg);

  fsai::control::runtime::Ai2VcuCommandSet last_ai2vcu_commands{};
  bool has_last_ai_command = false;

  const uint64_t stale_threshold_ns =
      fsai_clock_from_seconds(kCommandStaleSeconds);


  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      Graphics_HandleWindowEvent(&graphics, &event);
      if (event.type == SDL_KEYDOWN && event.key.repeat == 0) {
        switch (event.key.keysym.sym) {
          case SDLK_ESCAPE:
            user_input.PublishEmergencyStop();
            break;
          case SDLK_1:
          case SDLK_KP_1:
            user_input.PublishMissionOverride(kMissionOptions[0].descriptor);
            break;
          case SDLK_2:
          case SDLK_KP_2:
            user_input.PublishMissionOverride(kMissionOptions[1].descriptor);
            break;
          case SDLK_3:
          case SDLK_KP_3:
            user_input.PublishMissionOverride(kMissionOptions[2].descriptor);
            break;
          case SDLK_4:
          case SDLK_KP_4:
            user_input.PublishMissionOverride(kMissionOptions[3].descriptor);
            break;
          default:
            break;
        }
      }
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }
    if (user_input.ConsumeEmergencyStop()) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Emergency stop requested; shutting down simulator");
      running = false;
    }
    if (!running) {
      break;
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    can_interface.Poll(fsai_clock_now());

    float autopThrottle = world.throttleInput;
    float autopBrake = world.brakeInput;
    float autopSteer = world.steeringAngle;
    if (world.useController) {
      float raThrottle = 0.0f;
      float raSteer = 0.0f;
      if (world.computeRacingControl(step_seconds, raThrottle, raSteer)) {
        autopSteer = raSteer;
        if (raThrottle >= 0.0f) {
          autopThrottle = raThrottle;
          autopBrake = 0.0f;
        } else {
          autopThrottle = 0.0f;
          autopBrake = -raThrottle;
        }
      }
    }

    const VehicleState& veh_state = world.vehicle_state();
    const double vx = veh_state.velocity.x();
    const double vy = veh_state.velocity.y();
    const float speed_mps = static_cast<float>(std::sqrt(vx * vx + vy * vy));
    const auto can_vehicle_state = can_interface.LatestVehicleState();
    float measured_speed_mps = speed_mps;
    if (can_vehicle_state) {
      measured_speed_mps = std::max(can_vehicle_state->vx, measured_speed_mps);
    }

    fsai::sim::svcu::CommandPacket command_packet{};
    while (auto bytes = command_rx.receive(&command_packet, sizeof(command_packet))) {
      if (*bytes == sizeof(command_packet)) {
        io_bus->push_command(command_packet);
      }
    }

    const uint64_t now_ns = fsai_clock_advance(step_ns);

    auto compute_age_seconds = [&](uint64_t timestamp_ns) {
      if (timestamp_ns == 0 || now_ns == 0 || now_ns < timestamp_ns) {
        return std::numeric_limits<double>::infinity();
      }
      return static_cast<double>(now_ns - timestamp_ns) * 1e-9;
    };

    float appliedThrottle = autopThrottle;
    float appliedBrake = autopBrake;
    float appliedSteer = autopSteer;
    bool ai_command_applied = false;
    auto latest_command = io_bus->latest_command();
    const bool ai_command_enabled = latest_command && latest_command->enabled != 0;
    if (latest_command) {
      if (latest_command->t_ns <= now_ns &&
          now_ns - latest_command->t_ns <= stale_threshold_ns) {
        appliedThrottle = latest_command->throttle;
        appliedBrake = latest_command->brake;
        appliedSteer = latest_command->steer_rad;
        ai_command_applied = true;
      }
    }

    fsai::sim::app::RuntimeTelemetry::TimedSample<fsai::types::ControlCmd>
        ai_command_sample{};
    if (latest_command) {
      ai_command_sample.valid = true;
      ai_command_sample.value.steer_rad = latest_command->steer_rad;
      ai_command_sample.value.throttle = latest_command->throttle;
      ai_command_sample.value.brake = latest_command->brake;
      ai_command_sample.value.t_ns = latest_command->t_ns;
      ai_command_sample.age_s = compute_age_seconds(latest_command->t_ns);
    } else {
      ai_command_sample.age_s = std::numeric_limits<double>::infinity();
    }
    const bool fallback_to_manual = ai_command_enabled && !ai_command_applied;

    appliedThrottle = std::clamp(appliedThrottle, 0.0f, 1.0f);
    appliedBrake = std::clamp(appliedBrake, 0.0f, 1.0f);

    fsai::types::ControlCmd control_cmd{};
    control_cmd.throttle = appliedThrottle;
    control_cmd.brake = appliedBrake;
    control_cmd.steer_rad = appliedSteer;
    control_cmd.t_ns = now_ns;

    if (now_ns - last_ai2vcu_tx_ns >= ai2vcu_period_ns) {
      const auto& mission_state = world.missionRuntime();
      fsai::control::runtime::Ai2VcuAdapter::AdapterTelemetry adapter_telemetry{};
      adapter_telemetry.measured_speed_mps = measured_speed_mps;
      adapter_telemetry.lap_counter = static_cast<uint8_t>(
          std::clamp(world.completedLaps(), 0, 15));
      adapter_telemetry.mission_laps_completed = static_cast<uint16_t>(
          std::min<std::size_t>(mission_state.completed_laps(),
                                 std::numeric_limits<uint16_t>::max()));
      adapter_telemetry.mission_laps_target = static_cast<uint16_t>(
          std::min<std::size_t>(mission_state.target_laps(),
                                 std::numeric_limits<uint16_t>::max()));
      adapter_telemetry.mission_selected = true;
      adapter_telemetry.mission_running =
          mission_state.run_status() == fsai::sim::MissionRunStatus::kRunning;
      adapter_telemetry.mission_finished =
          mission_state.run_status() == fsai::sim::MissionRunStatus::kCompleted ||
          mission_state.stop_commanded();
      if (world.public_ground_truth_enabled()) {
        const auto total_cones = world.getLeftCones().size() +
                                 world.getRightCones().size() +
                                 world.getStartCones().size();
        if (total_cones > 0) {
          adapter_telemetry.cones_count_all = static_cast<uint16_t>(
              std::min<size_t>(total_cones, std::numeric_limits<uint16_t>::max()));
        }
      }

      auto command_frames = ai2vcu_adapter.Adapt(
          control_cmd, can_interface.RawStatus(), adapter_telemetry);
      if (can_interface.Send(command_frames, now_ns)) {
        last_ai2vcu_tx_ns = now_ns;
        last_ai2vcu_commands = command_frames;
        has_last_ai_command = true;
      }
    }

    can_interface.Poll(now_ns);

    world.throttleInput = appliedThrottle;
    world.brakeInput = appliedBrake;
    world.steeringAngle = appliedSteer;

    vehicle_dynamics.setCommand(appliedThrottle, appliedBrake, appliedSteer);
    vehicle_dynamics.step(step_seconds);

    {
      fsai::time::ControlStageTimer control_timer("world_update");
      world.update(step_seconds);
    }
    const double sim_time_s = fsai_clock_to_seconds(now_ns);

    const auto& vehicle_state = world.vehicle_state();
    const auto& model = vehicle_dynamics.model();
    const auto& pt_status = model.lastPowertrainStatus();
    const auto& brake_status = model.lastBrakeStatus();
    const WheelsInfo& wheel_info = world.wheels_info();
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
    const auto& mission_state = world.missionRuntime();
    runtime_telemetry.mission.mission_time_s = mission_state.mission_time_seconds();
    runtime_telemetry.mission.straight_progress_m = mission_state.straight_line_progress_m();
    runtime_telemetry.mission.target_laps = mission_state.target_laps();
    runtime_telemetry.mission.completed_laps = static_cast<int>(mission_state.completed_laps());
    runtime_telemetry.mission.status = mission_state.run_status();
    runtime_telemetry.mission.stop_commanded = mission_state.stop_commanded();
    runtime_telemetry.mission.mission_name = mission_definition.descriptor.name;
    runtime_telemetry.mission.mission_short_name = mission_definition.descriptor.short_name;
    runtime_telemetry.mission.mission_type = mission_definition.descriptor.type;
    runtime_telemetry.mission.segments.clear();
    runtime_telemetry.mission.active_segment_index.reset();
    const auto& runtime_segments = mission_state.segments();
    const auto* active_segment = mission_state.current_segment();
    runtime_telemetry.mission.segments.reserve(runtime_segments.size());
    for (std::size_t i = 0; i < runtime_segments.size(); ++i) {
      const auto& segment = runtime_segments[i];
      fsai::sim::app::RuntimeTelemetry::MissionData::SegmentInfo info;
      info.type = segment.spec.type;
      info.completed_laps = segment.completed_laps;
      info.target_laps = segment.spec.laps;
      info.elapsed_time_s = segment.elapsed_time_s;
      runtime_telemetry.mission.segments.push_back(info);
      if (active_segment == &segment) {
        runtime_telemetry.mission.active_segment_index = i;
      }
    }
    if (const auto* segment = mission_state.current_segment()) {
      runtime_telemetry.mission.segment = segment->spec.type;
      runtime_telemetry.mission.segment_completed_laps = segment->completed_laps;
      runtime_telemetry.mission.segment_target_laps = segment->spec.laps;
    } else if (!mission_state.segments().empty()) {
      const auto& last_segment = mission_state.segments().back();
      runtime_telemetry.mission.segment = last_segment.spec.type;
      runtime_telemetry.mission.segment_completed_laps = last_segment.spec.laps;
      runtime_telemetry.mission.segment_target_laps = last_segment.spec.laps;
    } else {
      runtime_telemetry.mission.segment = fsai::sim::MissionSegmentType::kTimed;
      runtime_telemetry.mission.segment_completed_laps = 0;
      runtime_telemetry.mission.segment_target_laps = 0;
    }
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

    runtime_telemetry.control.control_cmd = control_cmd;
    runtime_telemetry.control.applied_throttle = appliedThrottle;
    runtime_telemetry.control.applied_brake = appliedBrake;
    runtime_telemetry.control.applied_steer_rad = appliedSteer;
    runtime_telemetry.control.ai_command = ai_command_sample;
    runtime_telemetry.control.ai_command_enabled = ai_command_enabled;
    runtime_telemetry.control.ai_command_applied = ai_command_applied;
    runtime_telemetry.control.fallback_to_manual = fallback_to_manual;
    runtime_telemetry.control.has_last_command = has_last_ai_command;
    if (has_last_ai_command) {
      runtime_telemetry.control.last_command = last_ai2vcu_commands;
    }

    runtime_telemetry.mode.use_controller = world.useController != 0;
    runtime_telemetry.mode.runtime_mode = mode;
    {
    std::lock_guard<std::mutex> lock(shared_telemetry.mutex);
    shared_telemetry.position.x() = runtime_telemetry.pose.position_x_m;
    shared_telemetry.position.y() = runtime_telemetry.pose.position_y_m;
    // CRITICAL: Convert Telemetry degrees back to radians for Eigen rotation
    constexpr double kDegToRad = std::numbers::pi / 180.0;
    shared_telemetry.yaw_rad = runtime_telemetry.pose.yaw_deg * kDegToRad;
    }
    DrawMissionPanel(runtime_telemetry);
    DrawSimulationPanel(runtime_telemetry);
    DrawControlPanel(runtime_telemetry);
    DrawCanPanel(runtime_telemetry);
    DrawLogConsole();
    if (edge_preview_enabled && edge_preview.running()) {
      DrawEdgePreviewPanel(edge_preview, now_ns);
    }
    if (detection_preview_enabled && detection_preview.running()) {
      DrawDetectionPreviewPanel(detection_preview, now_ns);

    }

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

    BrakeSample brake_sample{};
    const double front_pct_raw = (std::max(0.0, brake_status.front_force) / front_max_force) * 100.0;
    const double rear_pct_raw = (std::max(0.0, brake_status.rear_force) / rear_max_force) * 100.0;
    brake_sample.front_pct = static_cast<float>(std::clamp(front_pct_raw +
                                           sample_noise(sensor_cfg.brake_pct.noise_std), 0.0, 100.0));
    brake_sample.rear_pct = static_cast<float>(std::clamp(rear_pct_raw +
                                          sample_noise(sensor_cfg.brake_pct.noise_std), 0.0, 100.0));
    brake_sample.front_req_pct = has_last_ai_command ? last_ai2vcu_commands.brake.front_pct : 0.0f;
    brake_sample.rear_req_pct = has_last_ai_command ? last_ai2vcu_commands.brake.rear_pct : 0.0f;
    brake_delay.push(now_ns, brake_sample);
    if (auto sample = brake_delay.poll(now_ns)) {
      brake_meas = *sample;
      has_brake_meas = true;
    }

    speed_delay.push(now_ns, static_cast<float>(std::max(0.0, actual_speed_kph +
                                     sample_noise(sensor_cfg.speed_kph.noise_std))));
    if (auto sample = speed_delay.poll(now_ns)) {
      speed_meas_kph = *sample;
      has_speed_meas = true;
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

    const float steer_for_msg = has_steer_meas ? steer_meas_deg : 0.0f;
    const std::array<float, 4> wheel_for_msg = has_wheel_meas ? wheel_meas_rpm
                                                              : std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f};
    const TorqueSample torque_for_msg = has_torque_meas ? torque_meas : TorqueSample{};
    const BrakeSample brake_for_msg = has_brake_meas ? brake_meas : BrakeSample{};
    const float speed_for_msg_kph = has_speed_meas ? speed_meas_kph : actual_speed_kph;

    fsai::sim::svcu::dbc::Vcu2AiStatus status_msg{};
    status_msg.handshake = true;
    status_msg.as_switch_on = true;
    status_msg.ts_switch_on = true;
    status_msg.go_signal = true;
    status_msg.as_state = world.useController
                              ? fsai::sim::svcu::dbc::AsState::kDriving
                              : fsai::sim::svcu::dbc::AsState::kReady;
    status_msg.steering_status = fsai::sim::svcu::dbc::SteeringStatus::kActive;

    fsai::sim::svcu::dbc::Vcu2AiSteer steer_msg{};
    steer_msg.angle_deg = steer_for_msg;
    steer_msg.angle_max_deg = fsai::sim::svcu::dbc::kMaxSteerDeg;
    steer_msg.angle_request_deg = has_last_ai_command
                                      ? last_ai2vcu_commands.steer.steer_deg
                                      : steer_for_msg;

    fsai::sim::svcu::dbc::Vcu2AiDrive front_drive_msg{};
    front_drive_msg.axle_trq_nm = torque_for_msg.front_nm;
    front_drive_msg.axle_trq_max_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
    front_drive_msg.axle_trq_request_nm = has_last_ai_command
        ? last_ai2vcu_commands.front_drive.axle_torque_request_nm
        : std::max(0.0f, torque_for_msg.front_nm);

    fsai::sim::svcu::dbc::Vcu2AiDrive rear_drive_msg{};
    rear_drive_msg.axle_trq_nm = torque_for_msg.rear_nm;
    rear_drive_msg.axle_trq_max_nm = fsai::sim::svcu::dbc::kMaxAxleTorqueNm;
    rear_drive_msg.axle_trq_request_nm = has_last_ai_command
        ? last_ai2vcu_commands.rear_drive.axle_torque_request_nm
        : std::max(0.0f, torque_for_msg.rear_nm);

    fsai::sim::svcu::dbc::Vcu2AiSpeeds speed_msg{};
    for (size_t i = 0; i < wheel_for_msg.size(); ++i) {
      speed_msg.wheel_rpm[i] = wheel_for_msg[i];
    }

    fsai::sim::svcu::dbc::Vcu2AiBrake brake_msg{};
    brake_msg.front_pct = brake_for_msg.front_pct;
    brake_msg.rear_pct = brake_for_msg.rear_pct;
    brake_msg.front_req_pct = brake_for_msg.front_req_pct;
    brake_msg.rear_req_pct = brake_for_msg.rear_req_pct;
    brake_msg.status_brk = fsai::sim::svcu::dbc::BrakeStatus::kReady;
    brake_msg.status_ebs = fsai::sim::svcu::dbc::EbsStatus::kArmed;

    fsai::sim::svcu::dbc::Vcu2LogDynamics1 dyn_msg{};
    dyn_msg.speed_actual_kph = speed_for_msg_kph;
    dyn_msg.speed_target_kph = has_last_ai_command
                                   ? last_ai2vcu_commands.status.veh_speed_demand_kph
                                   : speed_for_msg_kph;
    dyn_msg.steer_actual_deg = steer_for_msg;
    dyn_msg.steer_target_deg = has_last_ai_command
                                   ? last_ai2vcu_commands.steer.steer_deg
                                   : steer_for_msg;
    const float brake_target_pct = has_last_ai_command
        ? (last_ai2vcu_commands.brake.front_pct + last_ai2vcu_commands.brake.rear_pct) * 0.5f
        : (brake_for_msg.front_req_pct + brake_for_msg.rear_req_pct) * 0.5f;
    dyn_msg.brake_actual_pct = (brake_for_msg.front_pct + brake_for_msg.rear_pct) * 0.5f;
    dyn_msg.brake_target_pct = brake_target_pct;
    const double total_req_torque_nm = has_last_ai_command
        ? static_cast<double>(last_ai2vcu_commands.front_drive.axle_torque_request_nm +
                              last_ai2vcu_commands.rear_drive.axle_torque_request_nm)
        : 0.0;
    const double total_actual_torque_nm = std::abs(static_cast<double>(torque_for_msg.front_nm)) +
                                          std::abs(static_cast<double>(torque_for_msg.rear_nm));
    auto torque_to_pct = [](double torque_nm) {
      return static_cast<float>(std::clamp(torque_nm /
                                           (2.0 * fsai::sim::svcu::dbc::kMaxAxleTorqueNm) * 100.0,
                                           0.0, 255.0));
    };
    dyn_msg.drive_trq_actual_pct = torque_to_pct(total_actual_torque_nm);
    dyn_msg.drive_trq_target_pct = torque_to_pct(total_req_torque_nm);

    auto send_sim_frame = [&](const can_frame& frame) {
      if (!can_interface.SendSimulationFrame(frame)) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                             "Failed to send simulation CAN frame id=%u", frame.can_id);
      }
    };

    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_status(status_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_steer(steer_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_drive_front(front_drive_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_drive_rear(rear_drive_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_speeds(speed_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2ai_brake(brake_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_vcu2log_dynamics1(dyn_msg));
    send_sim_frame(fsai::sim::svcu::dbc::encode_ai2log_dynamics2(imu_meas));

    if (has_gps_meas) {
      can_interface.SetSimulationGpsSample(gps_meas.lat_deg, gps_meas.lon_deg,
                                          gps_meas.speed_mps);
    }

    can_interface.Poll(now_ns);

    fsai::sim::svcu::TelemetryPacket telemetry{};
    telemetry.t_ns = now_ns;
    telemetry.steer_angle_rad = appliedSteer;
    telemetry.front_axle_torque_nm = static_cast<float>(front_net_force * wheel_radius);
    telemetry.rear_axle_torque_nm = static_cast<float>(rear_net_force * wheel_radius);
    telemetry.wheel_speed_rpm[0] = wheel_info.lf_speed;
    telemetry.wheel_speed_rpm[1] = wheel_info.rf_speed;
    telemetry.wheel_speed_rpm[2] = wheel_info.lb_speed;
    telemetry.wheel_speed_rpm[3] = wheel_info.rb_speed;
    telemetry.brake_pressure_front_bar =
        static_cast<float>(std::max(0.0, brake_status.front_force) / 1000.0);
    telemetry.brake_pressure_rear_bar =
        static_cast<float>(std::max(0.0, brake_status.rear_force) / 1000.0);
    telemetry.imu_ax_mps2 = static_cast<float>(vehicle_state.acceleration.x());
    telemetry.imu_ay_mps2 = static_cast<float>(vehicle_state.acceleration.y());
    telemetry.imu_yaw_rate_rps = static_cast<float>(vehicle_state.rotation.z());
    telemetry.gps_lat_deg = static_cast<float>(
        metersToLatitude(vehicle_state.position.y()));
    telemetry.gps_lon_deg = static_cast<float>(
        metersToLongitude(vehicle_state.position.x()));
    telemetry.gps_speed_mps = static_cast<float>(
        std::sqrt(vehicle_state.velocity.x() * vehicle_state.velocity.x() +
                  vehicle_state.velocity.y() * vehicle_state.velocity.y()));
    telemetry.status_flags = static_cast<uint8_t>(world.useController ? 0x3 : 0x1);
    io_bus->publish_telemetry(telemetry);

    logger.logState(sim_time_s, world.vehicle_state());
    logger.logControl(sim_time_s, world.throttleInput, world.steeringAngle);

    if (stereo_source) {
      const auto& transform = world.vehicle_transform();
      stereo_source->setBodyPose(transform.position.x, transform.position.y,
                                 transform.position.z, transform.yaw);

      cone_positions.clear();
      if (world.render_ground_truth_enabled()) {
        const auto& left_cones = world.getLeftCones();
        const auto& right_cones = world.getRightCones();
        const auto& start_cones_for_render = world.getStartCones();
        cone_positions.reserve(left_cones.size() + right_cones.size() +
                               start_cones_for_render.size());

        const float color_scale = 1.0f / 255.0f;
        auto makeColor = [color_scale](int r, int g, int b) {
          return std::array<float, 3>{r * color_scale, g * color_scale,
                                      b * color_scale};
        };

        const auto start_body = makeColor(255, 140, 0);
        const auto start_stripe = makeColor(255, 255, 255);
        const auto left_body = makeColor(255, 214, 0);
        const auto left_stripe = makeColor(50, 50, 50);
        const auto right_body = makeColor(0, 102, 204);
        const auto right_stripe = makeColor(255, 255, 255);

        auto appendCone = [&](const Cone& cone) {
          fsai::io::camera::sim_stereo::SimConeInstance instance{};
          instance.position =
              {cone.position.x, cone.position.y, cone.position.z};
          instance.base_width = cone.radius * 2.0f;
          instance.height =
              (cone.type == ConeType::Start) ? fsai::sim::kLargeConeHeightMeters
                                             : fsai::sim::kSmallConeHeightMeters;
          switch (cone.type) {
            case ConeType::Start:
              instance.body_color = start_body;
              instance.stripe_color = start_stripe;
              instance.stripe_count = 2;
              break;
            case ConeType::Left:
              instance.body_color = left_body;
              instance.stripe_color = left_stripe;
              instance.stripe_count = 1;
              break;
            case ConeType::Right:
              instance.body_color = right_body;
              instance.stripe_color = right_stripe;
              instance.stripe_count = 1;
              break;
          }
          cone_positions.push_back(instance);
        };

        for (const auto& cone : left_cones) {
          appendCone(cone);
        }
        for (const auto& cone : right_cones) {
          appendCone(cone);
        }
        for (const auto& cone : start_cones_for_render) {
          appendCone(cone);
        }
      }
      stereo_source->setCones(cone_positions);
      const FsaiStereoFrame& frame = stereo_source->capture(now_ns);
      io_bus->publish_stereo_frame(frame);
    }

    std::shared_ptr<fsai::vision::DetectionRingBuffer> detection_buffer = fsai::vision::getActiveDetectionBuffer();
    if (detection_buffer == nullptr) {
      printf("Detection Buffer not initialised");
      exit(-1);
    }
    auto detections = detection_buffer->tryPop();
    if (detections != std::nullopt) {
        for (int i = 0; i < detections->n; i++) {
          world.coneDetections.push_back(detections->dets[i]);
      }
    }

    fsai::sim::app::GuiWorldAdapter gui_adapter(world);
    const auto world_snapshot = gui_adapter.snapshot();
    DrawWorldScene(&graphics, world_snapshot, runtime_telemetry);
    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), graphics.renderer);
    Graphics_Present(&graphics);

    SDL_Delay(static_cast<Uint32>(step_seconds * 1000.0));

    frame_counter++;
    if (mission_finished) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kInfo,
                           "Mission completion acknowledged; exiting runtime loop");
      running = false;
    }
  }
  if (vision_node) {
    vision_node->stop();
    fsai::sim::log::Logf(fsai::sim::log::Level::kInfo, "VisionNode stopped.");
  }
  detection_preview.stop();
  edge_preview.stop();
  stereo_display.reset();
  if (stereo_frame_buffer) {
    while (stereo_frame_buffer->tryPop().has_value()) {
    }
    fsai::vision::setActiveFrameRingBuffer(nullptr);
    stereo_frame_buffer.reset();
  }
  shutdown_imgui();
  Graphics_Cleanup(&graphics);
  fsai::sim::log::LogInfoToStdout(
      "Simulation complete. Car state log saved to CarStateLog.csv");
  return EXIT_SUCCESS;
}
