#pragma once

#include <memory>
#include <optional>
#include <string>

#include "can_defs.hpp"

namespace fsai::sim::svcu {

constexpr uint16_t kDefaultCanUdpPort = 47000;

class ICanLink {
 public:
  virtual ~ICanLink() = default;

  virtual bool open(const std::string& endpoint, bool enable_loopback) = 0;
  virtual void close() = 0;
  virtual bool send(const can_frame& frame) = 0;
  virtual std::optional<can_frame> receive() = 0;
};

std::unique_ptr<ICanLink> make_can_link(const std::string& endpoint_hint);
std::string canonicalize_can_endpoint(const std::string& endpoint_hint);
std::string default_can_endpoint();
bool is_udp_endpoint(const std::string& endpoint);

}  // namespace fsai::sim::svcu

