#pragma once

#include "can_link.hpp"

#include <optional>
#include <string>

namespace fsai::sim::svcu {

class SocketCanLink : public ICanLink {
 public:
  SocketCanLink();
  ~SocketCanLink() override;

  SocketCanLink(const SocketCanLink&) = delete;
  SocketCanLink& operator=(const SocketCanLink&) = delete;

  bool open(const std::string& iface, bool enable_loopback = false) override;
  void close() override;

  bool send(const can_frame& frame) override;
  std::optional<can_frame> receive() override;

  bool valid() const { return fd_ >= 0; }

 private:
  int fd_;
};

}  // namespace fsai::sim::svcu

