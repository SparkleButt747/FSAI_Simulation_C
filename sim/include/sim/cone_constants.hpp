#pragma once

namespace fsai::sim {

inline constexpr float kSmallConeBaseWidthMeters = 0.228f;
inline constexpr float kLargeConeBaseWidthMeters = 0.285f;
inline constexpr float kSmallConeRadiusMeters = kSmallConeBaseWidthMeters / 2.0f;
inline constexpr float kLargeConeRadiusMeters = kLargeConeBaseWidthMeters / 2.0f;
inline constexpr float kSmallConeMassKg = 0.45f;
inline constexpr float kLargeConeMassKg = 1.05f;

// Visualisation helpers. Expressed as ratios to avoid duplication across layers.
inline constexpr float kConeHeightToBaseRatio = 1.35f;
inline constexpr float kConeStripeHeightFraction = 0.18f;
inline constexpr float kConeStripeInsetFraction = 0.2f;

}  // namespace fsai::sim

