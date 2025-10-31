#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "can_defs.hpp"

namespace fsai::io::can {

constexpr uint16_t kDefaultCanUdpPort = 47000;

class ICanTransport {
 public:
  virtual ~ICanTransport() = default;

  virtual bool open(const std::string& endpoint, bool enable_loopback) = 0;
  virtual void close() = 0;
  virtual bool send(const can_frame& frame) = 0;
  virtual std::optional<can_frame> receive() = 0;
};

std::unique_ptr<ICanTransport> MakeTransport(const std::string& endpoint_hint);
std::string CanonicalizeEndpoint(const std::string& endpoint_hint);
std::string DefaultEndpoint();
bool IsUdpEndpoint(const std::string& endpoint);

struct EndpointPair {
  std::string ai_endpoint;
  std::string svcu_endpoint;
  bool is_udp{false};
};

EndpointPair ResolveEndpointPair(const std::string& endpoint_hint);

}  // namespace fsai::io::can
