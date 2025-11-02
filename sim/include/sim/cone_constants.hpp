#pragma once

namespace fsai::sim {

inline constexpr float kSmallConeBaseWidthMeters = 0.228f;
inline constexpr float kLargeConeBaseWidthMeters = 0.285f;
inline constexpr float kSmallConeRadiusMeters = kSmallConeBaseWidthMeters / 2.0f;
inline constexpr float kLargeConeRadiusMeters = kLargeConeBaseWidthMeters / 2.0f;
inline constexpr float kSmallConeHeightMeters = 0.325f;
inline constexpr float kLargeConeHeightMeters = 0.505f;
inline constexpr float kSmallConeMassKg = 0.45f;
inline constexpr float kLargeConeMassKg = 1.05f;

inline constexpr float kSmallConeHeightToBaseRatio =
    kSmallConeHeightMeters / kSmallConeBaseWidthMeters;
inline constexpr float kLargeConeHeightToBaseRatio =
    kLargeConeHeightMeters / kLargeConeBaseWidthMeters;

// Visualisation helpers.
inline constexpr float kConeStripeWidthFraction = 0.22f;

}  // namespace fsai::sim

