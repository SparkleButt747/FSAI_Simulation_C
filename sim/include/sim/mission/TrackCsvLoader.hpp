#pragma once

#include <filesystem>

#include "track/TrackGenerator.hpp"

namespace fsai::sim {

TrackResult LoadTrackFromCsv(const std::filesystem::path& csv_path);

}  // namespace fsai::sim

