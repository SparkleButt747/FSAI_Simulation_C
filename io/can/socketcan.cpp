#include "can_transport.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#endif

namespace fsai::io::can {

std::unique_ptr<ICanTransport> MakeUdpTransport();

#if defined(__linux__)

namespace {
class SocketCanTransport final : public ICanTransport {
 public:
  SocketCanTransport() = default;
  ~SocketCanTransport() override { close(); }

  bool open(const std::string& iface, bool enable_loopback) override {
    close();

    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      std::perror("socket(PF_CAN)");
      return false;
    }

    if (!enable_loopback) {
      int loopback = 0;
      if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback,
                       sizeof(loopback)) < 0) {
        std::perror("setsockopt(CAN_RAW_LOOPBACK)");
      }
    }

    struct ifreq ifr {
    };
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface.c_str());

    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      std::perror("ioctl(SIOCGIFINDEX)");
      close();
      return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::perror("bind(can)");
      close();
      return false;
    }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) {
      ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    }
    return true;
  }

  void close() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool send(const can_frame& frame) override {
    if (fd_ < 0) {
      return false;
    }
    const ssize_t written = ::write(fd_, &frame, sizeof(frame));
    return written == static_cast<ssize_t>(sizeof(frame));
  }

  std::optional<can_frame> receive() override {
    if (fd_ < 0) {
      return std::nullopt;
    }
    can_frame frame{};
    const ssize_t n = ::read(fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      return std::nullopt;
    }
    return frame;
  }

 private:
  int fd_{-1};
};
}  // namespace

std::unique_ptr<ICanTransport> MakeTransport(const std::string& endpoint_hint) {
  if (IsUdpEndpoint(endpoint_hint)) {
    return MakeUdpTransport();
  }
  return std::make_unique<SocketCanTransport>();
}

#else  // !__linux__

std::unique_ptr<ICanTransport> MakeTransport(const std::string& endpoint_hint) {
  (void)endpoint_hint;
  return MakeUdpTransport();
}

#endif

}  // namespace fsai::io::can
