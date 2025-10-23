#include "can_link.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <memory>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#if defined(__linux__)
#include "socketcan.hpp"
#endif

namespace fsai::sim::svcu {
namespace {

#pragma pack(push, 1)
struct PackedCanFrame {
  uint32_t id_be;
  uint8_t dlc;
  uint8_t data[CAN_MAX_DLEN];
};
#pragma pack(pop)
static_assert(sizeof(PackedCanFrame) == 13, "PackedCanFrame must be 13 bytes");

bool starts_with_ignore_case(const std::string& value, const char* prefix) {
  const std::size_t prefix_len = std::strlen(prefix);
  if (value.size() < prefix_len) {
    return false;
  }
  for (std::size_t i = 0; i < prefix_len; ++i) {
    const unsigned char lhs = static_cast<unsigned char>(value[i]);
    const unsigned char rhs = static_cast<unsigned char>(prefix[i]);
    if (std::tolower(lhs) != std::tolower(rhs)) {
      return false;
    }
  }
  return true;
}

uint16_t parse_port(const std::string& text, uint16_t fallback) {
  if (text.empty()) {
    return fallback;
  }
  char* end = nullptr;
  long value = std::strtol(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return fallback;
  }
  if (value <= 0 || value > 65535) {
    return fallback;
  }
  return static_cast<uint16_t>(value);
}

uint16_t extract_udp_port(const std::string& endpoint, uint16_t fallback) {
  std::string work = endpoint;
  if (starts_with_ignore_case(work, "udp:")) {
    work = work.substr(4);
    if (!work.empty() && work[0] == '/') {
      if (work.size() > 1 && work[1] == '/') {
        work = work.substr(2);
      } else {
        work = work.substr(1);
      }
    }
  }
  const auto colon_pos = work.rfind(':');
  std::string port_str;
  if (colon_pos != std::string::npos) {
    port_str = work.substr(colon_pos + 1);
  } else {
    port_str = work;
  }
  std::size_t start = 0;
  while (start < port_str.size() &&
         std::isspace(static_cast<unsigned char>(port_str[start]))) {
    ++start;
  }
  std::size_t end = port_str.size();
  while (end > start && std::isspace(static_cast<unsigned char>(port_str[end - 1]))) {
    --end;
  }
  port_str = port_str.substr(start, end - start);
  return parse_port(port_str, fallback);
}

class UdpCanLink final : public ICanLink {
 public:
  UdpCanLink() = default;
  ~UdpCanLink() override { close(); }

  bool open(const std::string& endpoint, bool /*enable_loopback*/) override {
    close();
    port_ = extract_udp_port(endpoint, kDefaultCanUdpPort);

    recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0) {
      return false;
    }
    send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd_ < 0) {
      close();
      return false;
    }

    int reuse = 1;
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#if defined(SO_REUSEPORT)
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port_);
    if (::bind(recv_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      close();
      return false;
    }

    peer_addr_ = addr;

    auto set_non_blocking = [](int fd) {
      int flags = ::fcntl(fd, F_GETFL, 0);
      if (flags >= 0) {
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
      }
    };
    set_non_blocking(recv_fd_);
    set_non_blocking(send_fd_);

    return true;
  }

  void close() override {
    if (recv_fd_ >= 0) {
      ::close(recv_fd_);
      recv_fd_ = -1;
    }
    if (send_fd_ >= 0) {
      ::close(send_fd_);
      send_fd_ = -1;
    }
    port_ = 0;
    std::memset(&peer_addr_, 0, sizeof(peer_addr_));
  }

  bool send(const can_frame& frame) override {
    if (send_fd_ < 0) {
      return false;
    }
    PackedCanFrame packed{};
    packed.id_be = htonl(frame.can_id);
    packed.dlc = static_cast<uint8_t>(std::min<int>(frame.can_dlc, CAN_MAX_DLEN));
    std::memcpy(packed.data, frame.data, packed.dlc);
    const ssize_t written = ::sendto(send_fd_, &packed, sizeof(packed), 0,
                                     reinterpret_cast<const sockaddr*>(&peer_addr_),
                                     sizeof(peer_addr_));
    return written == static_cast<ssize_t>(sizeof(packed));
  }

  std::optional<can_frame> receive() override {
    if (recv_fd_ < 0) {
      return std::nullopt;
    }
    PackedCanFrame packed{};
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    const ssize_t n = ::recvfrom(recv_fd_, &packed, sizeof(packed), 0,
                                 reinterpret_cast<sockaddr*>(&src), &src_len);
    if (n != static_cast<ssize_t>(sizeof(packed))) {
      return std::nullopt;
    }
    can_frame frame{};
    frame.can_id = ntohl(packed.id_be);
    frame.can_dlc = static_cast<uint8_t>(std::min<int>(packed.dlc, CAN_MAX_DLEN));
    std::memcpy(frame.data, packed.data, frame.can_dlc);
    return frame;
  }

 private:
  int recv_fd_{-1};
  int send_fd_{-1};
  uint16_t port_{0};
  sockaddr_in peer_addr_{};
};

}  // namespace

std::unique_ptr<ICanLink> make_can_link(const std::string& endpoint_hint) {
  const std::string canonical = canonicalize_can_endpoint(endpoint_hint);
  if (is_udp_endpoint(canonical)) {
    return std::make_unique<UdpCanLink>();
  }
#if defined(__linux__)
  (void)canonical;
  return std::make_unique<SocketCanLink>();
#else
  (void)canonical;
  return std::make_unique<UdpCanLink>();
#endif
}

std::string canonicalize_can_endpoint(const std::string& endpoint_hint) {
  if (endpoint_hint.empty()) {
#if defined(__linux__)
    return "vcan0";
#else
    return "udp:" + std::to_string(kDefaultCanUdpPort);
#endif
  }
  if (starts_with_ignore_case(endpoint_hint, "udp:")) {
    const uint16_t port = extract_udp_port(endpoint_hint, kDefaultCanUdpPort);
    return "udp:" + std::to_string(port);
  }
#if defined(__linux__)
  return endpoint_hint;
#else
  const uint16_t port = extract_udp_port(endpoint_hint, kDefaultCanUdpPort);
  return "udp:" + std::to_string(port);
#endif
}

std::string default_can_endpoint() { return canonicalize_can_endpoint(""); }

bool is_udp_endpoint(const std::string& endpoint) {
  return starts_with_ignore_case(endpoint, "udp:");
}

}  // namespace fsai::sim::svcu

