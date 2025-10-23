#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <optional>
#include <string>

namespace fsai::sim::svcu {

class SocketCan {
 public:
  SocketCan();
  ~SocketCan();

  SocketCan(const SocketCan&) = delete;
  SocketCan& operator=(const SocketCan&) = delete;

  bool open(const std::string& iface, bool enable_loopback = false);
  void close();

  bool send(const can_frame& frame) const;
  std::optional<can_frame> receive() const;

  bool valid() const { return fd_ >= 0; }

 private:
  int fd_;
};

}  // namespace fsai::sim::svcu
