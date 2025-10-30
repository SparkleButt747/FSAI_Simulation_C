#include <io/can/link.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace fsai::sim::svcu {

UdpEndpoint::UdpEndpoint() : fd_(-1), connected_(false) {}

UdpEndpoint::~UdpEndpoint() { close(); }

bool UdpEndpoint::bind(uint16_t port) {
  close();

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);

  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  connected_ = false;
  return true;
}

bool UdpEndpoint::connect(uint16_t port, const std::string& host) {
  close();

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(host.c_str());
  if (addr.sin_addr.s_addr == INADDR_NONE) {
    close();
    return false;
  }

  if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  connected_ = true;
  return true;
}

void UdpEndpoint::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  connected_ = false;
}

bool UdpEndpoint::send(const void* data, size_t size) const {
  if (fd_ < 0) {
    return false;
  }
  ssize_t sent = ::write(fd_, data, size);
  return sent == static_cast<ssize_t>(size);
}

std::optional<size_t> UdpEndpoint::receive(void* data, size_t size) const {
  if (fd_ < 0) {
    return std::nullopt;
  }
  ssize_t recvd = ::read(fd_, data, size);
  if (recvd <= 0) {
    return std::nullopt;
  }
  return static_cast<size_t>(recvd);
}

}  // namespace fsai::sim::svcu
