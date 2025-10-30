#include "loopback_can.hpp"

namespace fsai::sim::svcu {

bool LoopbackCanLink::open(const std::string&, bool) {
  while (!pending_.empty()) {
    pending_.pop();
  }
  return true;
}

void LoopbackCanLink::close() {
  while (!pending_.empty()) {
    pending_.pop();
  }
}

bool LoopbackCanLink::send(const can_frame& frame) {
  pending_.push(frame);
  return true;
}

std::optional<can_frame> LoopbackCanLink::receive() {
  if (pending_.empty()) {
    return std::nullopt;
  }
  can_frame frame = pending_.front();
  pending_.pop();
  return frame;
}

}  // namespace fsai::sim::svcu
