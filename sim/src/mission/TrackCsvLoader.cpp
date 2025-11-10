#include "sim/mission/TrackCsvLoader.hpp"

#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
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
  const std::string trimmed = Trim(token);
  if (trimmed.empty()) {
    throw std::runtime_error("Empty numeric field on line " + std::to_string(line_number));
  }
  char* end_ptr = nullptr;
  errno = 0;
  const float value = std::strtof(trimmed.c_str(), &end_ptr);
  if (end_ptr != trimmed.c_str() + trimmed.size() || errno == ERANGE) {
    throw std::runtime_error("Failed to parse float on line " + std::to_string(line_number));
  }
  return value;
}

Transform MakeTransform(float x, float z, float yaw_rad) {
  Transform transform{};
  transform.position.x = x;
  transform.position.y = 0.0f;
  transform.position.z = z;
  transform.yaw = yaw_rad;
  return transform;
}

struct ParsedRow {
  std::string tag;
  Transform transform;
  float x_variance;
  float y_variance;
  float xy_covariance;
};

std::vector<std::string> SplitRow(const std::string& line) {
  std::vector<std::string> columns;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    columns.push_back(Trim(cell));
  }
  return columns;
}

ParsedRow ParseRow(const std::vector<std::string>& columns, std::size_t line_number) {
  if (columns.size() != 7) {
    throw std::runtime_error("Expected 7 columns on line " + std::to_string(line_number));
  }

  ParsedRow row{};
  row.tag = columns[0];
  const float x_m = ParseFloat(columns[1], line_number);
  const float z_m = ParseFloat(columns[2], line_number);
  const float yaw_rad = ParseFloat(columns[3], line_number);
  row.transform = MakeTransform(x_m, z_m, yaw_rad);
  row.x_variance = ParseFloat(columns[4], line_number);
  row.y_variance = ParseFloat(columns[5], line_number);
  row.xy_covariance = ParseFloat(columns[6], line_number);
  (void)row.x_variance;
  (void)row.y_variance;
  (void)row.xy_covariance;
  return row;
}

float DistanceSquared(const Transform& a, const Transform& b) {
  const float dx = b.position.x - a.position.x;
  const float dz = b.position.z - a.position.z;
  return dx * dx + dz * dz;
}

std::vector<Transform> BuildCheckpointsFromBoundaries(const std::vector<Transform>& left,
                                                      const std::vector<Transform>& right) {
  std::vector<Transform> checkpoints;
  if (left.empty() || right.empty()) {
    return checkpoints;
  }

  std::vector<bool> right_used(right.size(), false);
  checkpoints.reserve(std::min(left.size(), right.size()));

  for (const auto& left_cone : left) {
    float best_distance = std::numeric_limits<float>::infinity();
    std::size_t best_index = right.size();
    for (std::size_t i = 0; i < right.size(); ++i) {
      if (right_used[i]) {
        continue;
      }
      const float distance = DistanceSquared(left_cone, right[i]);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = i;
      }
    }

    if (best_index == right.size()) {
      continue;
    }

    right_used[best_index] = true;
    Transform checkpoint{};
    checkpoint.position.x = 0.5f * (left_cone.position.x + right[best_index].position.x);
    checkpoint.position.y = 0.0f;
    checkpoint.position.z = 0.5f * (left_cone.position.z + right[best_index].position.z);
    const float dx = right[best_index].position.x - left_cone.position.x;
    const float dz = right[best_index].position.z - left_cone.position.z;
    checkpoint.yaw = std::atan2(dz, dx);
    checkpoints.push_back(checkpoint);
  }

  return checkpoints;
}

}  // namespace

TrackResult LoadTrackFromCsv(const std::filesystem::path& csv_path) {
  std::ifstream input(csv_path);
  if (!input.is_open()) {
    throw std::runtime_error("Failed to open track CSV: " + csv_path.string());
  }

  TrackResult track;
  std::vector<Transform> left_primary;
  std::vector<Transform> right_primary;
  std::vector<Transform> collected_midpoints;
  std::optional<Transform> car_start;
  std::string line;
  std::size_t line_number = 0;
  bool header_consumed = false;
  while (std::getline(input, line)) {
    ++line_number;
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || StartsWith(trimmed, '#')) {
      continue;
    }

    const std::vector<std::string> columns = SplitRow(line);
    if (!header_consumed && !columns.empty() && columns[0] == "tag") {
      header_consumed = true;
      continue;
    }

    const ParsedRow row = ParseRow(columns, line_number);

    if (row.tag == "blue") {
      track.leftCones.push_back(row.transform);
      left_primary.push_back(row.transform);
    } else if (row.tag == "yellow") {
      track.rightCones.push_back(row.transform);
      right_primary.push_back(row.transform);
    } else if (row.tag == "big_orange") {
      track.startCones.push_back(row.transform);
    } else if (row.tag == "orange") {
      if (row.transform.position.z >= 0.0f) {
        track.leftCones.push_back(row.transform);
      } else {
        track.rightCones.push_back(row.transform);
      }
    } else if (row.tag == "midpoint") {
      collected_midpoints.push_back(row.transform);
    } else if (row.tag == "car_start") {
      car_start = row.transform;
    } else {
      throw std::runtime_error("Unrecognized element tag '" + row.tag +
                               "' on line " + std::to_string(line_number));
    }
  }

  std::vector<Transform> checkpoints;
  checkpoints.reserve(collected_midpoints.size() + (car_start.has_value() ? 1 : 0));
  if (car_start.has_value()) {
    checkpoints.push_back(*car_start);
  }
  checkpoints.insert(checkpoints.end(), collected_midpoints.begin(), collected_midpoints.end());

  if (checkpoints.empty()) {
    checkpoints = BuildCheckpointsFromBoundaries(left_primary, right_primary);
  }

  if (checkpoints.empty()) {
    throw std::runtime_error("Track CSV did not define any checkpoints or derivable midpoints: " +
                             csv_path.string());
  }

  track.checkpoints = std::move(checkpoints);
  return track;
}

}  // namespace fsai::sim

