#include "vision/detect.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

namespace fsai::vision {
namespace {
inline int intensityAt(const FsaiFrame& frame, int x, int y) {
  const std::size_t offset = static_cast<std::size_t>(y) *
                                 static_cast<std::size_t>(frame.stride) +
                             static_cast<std::size_t>(x) * 3u;
  const uint8_t* pixel = frame.data + offset;
  return static_cast<int>(pixel[0]) + static_cast<int>(pixel[1]) +
         static_cast<int>(pixel[2]);
}

bool isValidFrame(const FsaiFrame& frame) {
  return frame.data != nullptr && frame.w > 2 && frame.h > 2 && frame.stride > 0 &&
         frame.stride >= frame.w * 3;
}
}  // namespace

EdgeDetector::EdgeDetector(EdgeDetectorOptions options) : options_(options) {
  if (options_.gradient_threshold < 1) {
    options_.gradient_threshold = 1;
  }
  if (options_.min_component_pixels < 1) {
    options_.min_component_pixels = 1;
  }
  if (options_.confidence_normalizer <= 0.0f) {
    options_.confidence_normalizer = 1.0f;
  }
}

std::vector<BoxBound> EdgeDetector::process(const FsaiStereoFrame& frame) const {
  const FsaiFrame& left = frame.left;
  if (!isValidFrame(left)) {
    return {};
  }

  const int width = left.w;
  const int height = left.h;
  const std::size_t pixel_count = static_cast<std::size_t>(width) *
                                  static_cast<std::size_t>(height);
  std::vector<uint8_t> edges(pixel_count, 0);

  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      const int gx = intensityAt(left, x + 1, y) - intensityAt(left, x - 1, y);
      const int gy = intensityAt(left, x, y + 1) - intensityAt(left, x, y - 1);
      const int gradient = std::abs(gx) + std::abs(gy);
      if (gradient >= options_.gradient_threshold) {
        edges[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
              static_cast<std::size_t>(x)] = 1;
      }
    }
  }

  std::vector<BoxBound> detections;
  detections.reserve(16);
  std::vector<uint8_t> visited(pixel_count, 0);
  std::queue<std::pair<int, int>> work;
  constexpr std::array<std::pair<int, int>, 4> kOffsets{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) *
                                    static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(x);
      if (!edges[index] || visited[index]) {
        continue;
      }

      visited[index] = 1;
      work.emplace(x, y);

      int min_x = x;
      int max_x = x;
      int min_y = y;
      int max_y = y;
      int count = 0;

      while (!work.empty()) {
        const auto [cx, cy] = work.front();
        work.pop();
        ++count;
        min_x = std::min(min_x, cx);
        max_x = std::max(max_x, cx);
        min_y = std::min(min_y, cy);
        max_y = std::max(max_y, cy);

        for (const auto [dx, dy] : kOffsets) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const std::size_t nindex = static_cast<std::size_t>(ny) *
                                         static_cast<std::size_t>(width) +
                                     static_cast<std::size_t>(nx);
          if (!edges[nindex] || visited[nindex]) {
            continue;
          }
          visited[nindex] = 1;
          work.emplace(nx, ny);
        }
      }

      if (count < options_.min_component_pixels) {
        continue;
      }

      BoxBound bound{};
      bound.x = min_x;
      bound.y = min_y;
      bound.w = static_cast<float>(max_x - min_x + 1);
      bound.h = static_cast<float>(max_y - min_y + 1);
      bound.conf = std::min(1.0f, static_cast<float>(count) / options_.confidence_normalizer);
      bound.side = fsai::types::ConeSide::FSAI_CONE_UNKNOWN;
      detections.push_back(bound);
    }
  }

  return detections;
}

}  // namespace fsai::vision

