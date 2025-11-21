#pragma once

#include "can_link.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace fsai::sim::svcu {

#if defined(__APPLE__)
constexpr std::size_t kShmRingSize = 32;
struct ShmRing {
  std::size_t head{0};
  std::size_t tail{0};
  can_frame frames[kShmRingSize];
};
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

