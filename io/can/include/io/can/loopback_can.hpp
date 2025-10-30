#pragma once

#include <queue>
#include <optional>
#include <string>

#include "can_defs.hpp"
#include "can_link.hpp"

namespace fsai::sim::svcu {

class LoopbackCanLink : public ICanLink {
 public:
  LoopbackCanLink() = default;
  ~LoopbackCanLink() override = default;

  bool open(const std::string& endpoint, bool enable_loopback) override;
  void close() override;
  bool send(const can_frame& frame) override;
  std::optional<can_frame> receive() override;

 private:
  std::queue<can_frame> pending_{};
};

}  // namespace fsai::sim::svcu
