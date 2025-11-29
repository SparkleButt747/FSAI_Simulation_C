#include "socketcan.hpp"

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
#include <iostream>

namespace fsai::sim::svcu {

SocketCanLink::SocketCanLink() : fd_(-1) {}

SocketCanLink::~SocketCanLink() { close(); }

bool SocketCanLink::open(const std::string& iface, bool enable_loopback) {
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

  int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }
  return true;
}

void SocketCanLink::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanLink::send(const can_frame& frame) {
  if (fd_ < 0) {
    return false;
  }
  ssize_t n = ::write(fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

std::optional<can_frame> SocketCanLink::receive() {
  if (fd_ < 0) {
    return std::nullopt;
  }
  can_frame frame{};
  ssize_t n = ::read(fd_, &frame, sizeof(frame));
  if (n == static_cast<ssize_t>(sizeof(frame))) {
    return frame;
  }
  return std::nullopt;
}

}  // namespace fsai::sim::svcu

#else

namespace fsai::sim::svcu {

SocketCanLink::SocketCanLink() : fd_(-1) {}

SocketCanLink::~SocketCanLink() { close(); }

bool SocketCanLink::open(const std::string&, bool) { return false; }

void SocketCanLink::close() { fd_ = -1; }

bool SocketCanLink::send(const can_frame&) { return false; }

std::optional<can_frame> SocketCanLink::receive() { return std::nullopt; }

}  // namespace fsai::sim::svcu

#endif

