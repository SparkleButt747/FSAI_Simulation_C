#include "checkpoint_matching.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace fsai::control {
namespace {

float DistanceSquared(const Transform& a, const Transform& b) {
  const float dx = b.position.x - a.position.x;
  const float dz = b.position.z - a.position.z;
  return dx * dx + dz * dz;
}

Transform MakeCheckpoint(const Transform& left, const Transform& right) {
  Transform checkpoint{};
  checkpoint.position.x = 0.5f * (left.position.x + right.position.x);
  checkpoint.position.y = 0.0f;
  checkpoint.position.z = 0.5f * (left.position.z + right.position.z);
  const float dx = right.position.x - left.position.x;
  const float dz = right.position.z - left.position.z;
  checkpoint.yaw = std::atan2(dz, dx);
  return checkpoint;
}

struct MatchRecord {
  std::size_t left_index{0};
  std::size_t right_index{0};
};

}  // namespace

std::vector<MatchedGate> MatchConesToGates(const std::vector<Transform>& left,
                                           const std::vector<Transform>& right) {
  const std::size_t target = std::min(left.size(), right.size());
  if (target == 0U) {
    return {};
  }

  std::vector<bool> left_used(left.size(), false);
  std::vector<bool> right_used(right.size(), false);
  std::vector<MatchRecord> matches;
  matches.reserve(target);

  while (matches.size() < target) {
    std::vector<std::size_t> left_best(left.size(), right.size());
    std::vector<float> left_best_dist(left.size(), std::numeric_limits<float>::infinity());
    for (std::size_t li = 0; li < left.size(); ++li) {
      if (left_used[li]) {
        continue;
      }
      for (std::size_t ri = 0; ri < right.size(); ++ri) {
        if (right_used[ri]) {
          continue;
        }
        const float distance = DistanceSquared(left[li], right[ri]);
        if (distance < left_best_dist[li]) {
          left_best_dist[li] = distance;
          left_best[li] = ri;
        }
      }
    }

    std::vector<std::size_t> right_best(right.size(), left.size());
    std::vector<float> right_best_dist(right.size(), std::numeric_limits<float>::infinity());
    for (std::size_t ri = 0; ri < right.size(); ++ri) {
      if (right_used[ri]) {
        continue;
      }
      for (std::size_t li = 0; li < left.size(); ++li) {
        if (left_used[li]) {
          continue;
        }
        const float distance = DistanceSquared(left[li], right[ri]);
        if (distance < right_best_dist[ri]) {
          right_best_dist[ri] = distance;
          right_best[ri] = li;
        }
      }
    }

    bool matched_this_round = false;
    for (std::size_t li = 0; li < left.size() && matches.size() < target; ++li) {
      if (left_used[li]) {
        continue;
      }
      const std::size_t candidate = left_best[li];
      if (candidate >= right.size() || right_used[candidate]) {
        continue;
      }
      if (right_best[candidate] != li) {
        continue;
      }
      left_used[li] = true;
      right_used[candidate] = true;
      matches.push_back(MatchRecord{li, candidate});
      matched_this_round = true;
    }

    if (matches.size() >= target) {
      break;
    }

    if (!matched_this_round) {
      float best_distance = std::numeric_limits<float>::infinity();
      std::size_t best_left = left.size();
      std::size_t best_right = right.size();
      for (std::size_t li = 0; li < left.size(); ++li) {
        if (left_used[li]) {
          continue;
        }
        for (std::size_t ri = 0; ri < right.size(); ++ri) {
          if (right_used[ri]) {
            continue;
          }
          const float distance = DistanceSquared(left[li], right[ri]);
          if (distance < best_distance) {
            best_distance = distance;
            best_left = li;
            best_right = ri;
          }
        }
      }
      if (best_left == left.size() || best_right == right.size()) {
        break;
      }
      left_used[best_left] = true;
      right_used[best_right] = true;
      matches.push_back(MatchRecord{best_left, best_right});
    }
  }

  std::sort(matches.begin(), matches.end(), [](const MatchRecord& a, const MatchRecord& b) {
    return a.left_index < b.left_index;
  });

  std::vector<MatchedGate> result;
  result.reserve(matches.size());
  for (const MatchRecord& record : matches) {
    MatchedGate gate;
    gate.left = left[record.left_index];
    gate.right = right[record.right_index];
    gate.checkpoint = MakeCheckpoint(gate.left, gate.right);
    result.push_back(gate);
  }
  return result;
}

}  // namespace fsai::control
