#include "Telemetry.hpp"
#include <cstdio>
#include <cmath>

#include "fsai_clock.h"

namespace {
constexpr uint64_t kTelemetryLogPeriodNs = 500'000'000ULL;  // 0.5 s cadence
}  // namespace

void Telemetry_Update(const VehicleState& carState, const Transform& carTransform,
                      uint64_t simTimeNs, double lapTimeSeconds,
                      double totalDistance, int lapCount) {
    static uint64_t lastLogNs = 0;

    if (lastLogNs != 0 && (simTimeNs - lastLogNs) < kTelemetryLogPeriodNs) {
        return;
    }

    lastLogNs = simTimeNs;

    const double simTimeSeconds = fsai_clock_to_seconds(simTimeNs);
    const double speed = std::sqrt(carState.velocity.x() * carState.velocity.x() +
                                   carState.velocity.y() * carState.velocity.y() +
                                   carState.velocity.z() * carState.velocity.z());

    std::printf("[telemetry] t=%.2fs lap=%.2fs dist=%.1fm lap#=%d speed=%.2fm/s yaw=%.2f\n",
                simTimeSeconds, lapTimeSeconds, totalDistance, lapCount, speed, carState.yaw);
    std::printf("             pose=(%.2f, %.2f, %.2f) heading=%.2f\n",
                carTransform.position.x, carTransform.position.y, carTransform.position.z, carTransform.yaw);
}
