#pragma once

#include "can_link.hpp"

#include <optional>
#include <string>

namespace fsai::sim::svcu {

#if defined(__APPLE__)
struct ShmRing;
#endif

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
#if defined(__APPLE__)
  struct ShmRing* shared_;
  bool created_;
#endif
};

}  // namespace fsai::sim::svcu

