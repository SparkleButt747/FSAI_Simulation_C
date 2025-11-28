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

#elif defined(__APPLE__)

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <string>

namespace fsai::sim::svcu {

namespace {
std::string make_shm_name(const std::string& iface) {
  std::string name = "/fsai_socketcan_" + iface;
  if (name.size() > 31) {
    name.resize(31);
  }
  return name;
}

}  // namespace

SocketCanLink::SocketCanLink() : fd_(-1), shared_(nullptr), created_(false) {}

SocketCanLink::~SocketCanLink() { close(); }

bool SocketCanLink::open(const std::string& iface, bool) {
  close();
  const std::string name = make_shm_name(iface);
  fd_ = ::shm_open(name.c_str(), O_RDWR | O_CREAT, 0600);
  if (fd_ < 0) {
    return false;
  }
  created_ = (::ftruncate(fd_, sizeof(ShmRing)) == 0);
  void* addr = ::mmap(nullptr, sizeof(ShmRing), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (addr == MAP_FAILED) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  shared_ = static_cast<ShmRing*>(addr);
  if (created_) {
    std::memset(shared_, 0, sizeof(ShmRing));
  }
  return true;
}

void SocketCanLink::close() {
  if (shared_) {
    ::munmap(shared_, sizeof(ShmRing));
    shared_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanLink::send(const can_frame& frame) {
  if (!shared_) {
    return false;
  }
  shared_->frames[shared_->head] = frame;
  shared_->head = (shared_->head + 1) % kShmRingSize;
  if (shared_->head == shared_->tail) {
    shared_->tail = (shared_->tail + 1) % kShmRingSize;
  }
  return true;
}

std::optional<can_frame> SocketCanLink::receive() {
  if (!shared_ || shared_->tail == shared_->head) {
    return std::nullopt;
  }
  can_frame frame = shared_->frames[shared_->tail];
  shared_->tail = (shared_->tail + 1) % kShmRingSize;
  return frame;
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

