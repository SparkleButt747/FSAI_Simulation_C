#pragma once

#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "Vector.h"
#include "common/types.h"

namespace fsai::world {

struct WorldDebugPacket {
  std::vector<Vector3> start_cones;
  std::vector<Vector3> left_cones;
  std::vector<Vector3> right_cones;
  std::vector<Vector3> checkpoints;
  std::vector<std::pair<Vector2, Vector2>> controller_path;
  std::vector<FsaiConeDet> detections;
};

class IWorldDebugPublisher {
 public:
  virtual ~IWorldDebugPublisher() = default;

  virtual void set_debug_mode(bool enabled) = 0;
  virtual void publish(const WorldDebugPacket& packet) = 0;
  virtual std::optional<WorldDebugPacket> latest_packet() const = 0;
};

class InProcessWorldDebugPublisher final : public IWorldDebugPublisher {
 public:
  using Subscriber = std::function<void(const WorldDebugPacket&)>;

  void set_debug_mode(bool enabled) override { debug_mode_ = enabled; }

  void set_subscriber(Subscriber subscriber) { subscriber_ = std::move(subscriber); }

  void publish(const WorldDebugPacket& packet) override {
    if (!debug_mode_) {
      return;
    }

    latest_ = packet;
    if (subscriber_) {
      subscriber_.value()(packet);
    }
  }

  std::optional<WorldDebugPacket> latest_packet() const override { return latest_; }

 private:
  bool debug_mode_{false};
  std::optional<WorldDebugPacket> latest_{};
  std::optional<Subscriber> subscriber_{};
};

}  // namespace fsai::world

