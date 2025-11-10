#include "sim/mission/TrackCsvLoader.hpp"

#include <charconv>
#include <cstddef>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace fsai::sim {
namespace {

bool StartsWith(std::string_view text, char ch) {
  if (text.empty()) {
    return false;
  }
  return text.front() == ch;
}

std::string Trim(std::string_view text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string_view::npos) {
    return std::string();
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return std::string(text.substr(begin, end - begin + 1));
}

float ParseFloat(std::string_view token, std::size_t line_number) {
  token = Trim(token);
  if (token.empty()) {
    throw std::runtime_error("Empty numeric field on line " + std::to_string(line_number));
  }
  float value = 0.0f;
  const auto* begin = token.data();
  const auto* end = token.data() + token.size();
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end) {
    throw std::runtime_error("Failed to parse float on line " + std::to_string(line_number));
  }
  return value;
}

Transform MakeTransform(float x, float y, float z, float yaw_deg) {
  Transform transform{};
  transform.position.x = x;
  transform.position.y = y;
  transform.position.z = z;
  transform.yaw = yaw_deg * (std::numbers::pi_v<float> / 180.0f);
  return transform;
}

}  // namespace

TrackResult LoadTrackFromCsv(const std::filesystem::path& csv_path) {
  std::ifstream input(csv_path);
  if (!input.is_open()) {
    throw std::runtime_error("Failed to open track CSV: " + csv_path.string());
  }

  TrackResult track;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || StartsWith(trimmed, '#')) {
      continue;
    }

    std::vector<std::string> columns;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      columns.push_back(Trim(cell));
    }

    if (columns.size() != 5) {
      throw std::runtime_error("Expected 5 columns on line " + std::to_string(line_number));
    }

    const std::string& type = columns[0];
    const float x_m = ParseFloat(columns[1], line_number);
    const float y_m = ParseFloat(columns[2], line_number);
    const float z_m = ParseFloat(columns[3], line_number);
    const float yaw_deg = ParseFloat(columns[4], line_number);
    Transform transform = MakeTransform(x_m, y_m, z_m, yaw_deg);

    if (type == "start") {
      track.startCones.push_back(transform);
    } else if (type == "left") {
      track.leftCones.push_back(transform);
    } else if (type == "right") {
      track.rightCones.push_back(transform);
    } else if (type == "checkpoint") {
      track.checkpoints.push_back(transform);
    } else {
      throw std::runtime_error("Unrecognized element type '" + type + "' on line " +
                               std::to_string(line_number));
    }
  }

  if (track.checkpoints.empty()) {
    throw std::runtime_error("Track CSV did not define any checkpoints: " + csv_path.string());
  }

  return track;
}

}  // namespace fsai::sim

