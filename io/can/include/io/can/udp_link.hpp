#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace fsai::sim::svcu {

constexpr uint16_t kDefaultCommandPort = 47001;
constexpr uint16_t kDefaultTelemetryPort = 47002;

#pragma pack(push, 1)
struct CommandPacket {
  uint64_t t_ns;
  float steer_rad;
  float throttle;
  float brake;
  uint8_t enabled;
  uint8_t reserved[3];
};
static_assert(sizeof(CommandPacket) == 24, "CommandPacket size unexpected");

struct TelemetryPacket {
  uint64_t t_ns;
  float steer_angle_rad;
  float front_axle_torque_nm;
  float rear_axle_torque_nm;
  float wheel_speed_rpm[4];
  float brake_pressure_front_bar;
  float brake_pressure_rear_bar;
  float imu_ax_mps2;
  float imu_ay_mps2;
  float imu_yaw_rate_rps;
  float gps_lat_deg;
  float gps_lon_deg;
  float gps_speed_mps;
  uint8_t status_flags;
  uint8_t reserved[3];
};
static_assert(sizeof(TelemetryPacket) == 72, "TelemetryPacket size unexpected");
#pragma pack(pop)

class UdpEndpoint {
 public:
  UdpEndpoint();
  ~UdpEndpoint();

  UdpEndpoint(const UdpEndpoint&) = delete;
  UdpEndpoint& operator=(const UdpEndpoint&) = delete;

  bool bind(uint16_t port);
  bool connect(uint16_t port, const std::string& host = "127.0.0.1");
  void close();

  bool send(const void* data, size_t size) const;
  std::optional<size_t> receive(void* data, size_t size) const;

  bool valid() const { return fd_ >= 0; }

 private:
  int fd_;
  bool connected_;
};

}  // namespace fsai::sim::svcu
