#pragma once

#include <utility>
#include <vector>

#include "Vector.h"
#include "common/types.h"

namespace fsai::world {

struct WorldDebugPacket {
  std::vector<Vector3> start_cones;
  std::vector<Vector3> left_cones;
  std::vector<Vector3> right_cones;
  std::vector<Vector3> orange_cones;
  std::vector<Vector3> checkpoints;
  std::vector<std::pair<Vector2, Vector2>> controller_path_edges;
  std::vector<FsaiConeDet> detections;
};

class IWorldDebugPublisher {
 public:
  virtual ~IWorldDebugPublisher() = default;
  virtual void publish_debug_packet(const WorldDebugPacket& packet) = 0;
};

}  // namespace fsai::world

