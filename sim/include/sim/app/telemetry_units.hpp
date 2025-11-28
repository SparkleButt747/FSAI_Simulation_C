#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <string_view>

#include "logging.hpp"

namespace fsai::sim::app {

// TelemetryUnits centralizes the Velox telemetry contract (XY plane, radians)
// and the OpenGL renderer/world expectations (XZ plane, degrees when exposed to the UI).
// Conversions also enforce finite numbers and range checks so bad units surface
// as logged errors immediately.
struct TelemetryUnits {
  static constexpr double kRadToDeg = 180.0 / std::numbers::pi;
  static constexpr double kDegToRad = std::numbers::pi / 180.0;
  static constexpr double kMaxTelemetryAngleRad = std::numbers::pi * 4.0;
  static constexpr double kMaxTelemetryAngleDeg = kMaxTelemetryAngleRad * kRadToDeg;
  static constexpr double kMaxTelemetrySteerRad = std::numbers::pi / 2.0;
  static constexpr double kMaxAngularRateRadPerSec = std::numbers::pi * 20.0;

  static bool ValidateFinite(std::string_view label, double value);
  static bool ValidateMagnitude(std::string_view label, double value, double max_abs);
  static bool ValidateRadians(std::string_view label, double value,
                              double max_abs = kMaxTelemetryAngleRad);
  static bool ValidateSteerAngle(std::string_view label, double value,
                                 double max_abs = kMaxTelemetrySteerRad);
  static bool ValidateAngularRate(std::string_view label, double value,
                                  double max_abs = kMaxAngularRateRadPerSec);

  static double RadToDeg(double rad, std::string_view label,
                         double max_abs = kMaxTelemetryAngleRad);
  static double DegToRad(double deg, std::string_view label,
                         double max_abs = kMaxTelemetryAngleDeg);

  static Eigen::Vector3d VeloxPlanarPosition(double x, double y);
  static Eigen::Vector3d VeloxPlanarVelocity(double x, double y);
  static Eigen::Vector3d BodyAccelerationToGlobal(double longitudinal, double lateral,
                                                  double heading_rad);
  static Eigen::Vector3d OpenGLGroundPosition(double velox_x, double velox_y);
};

inline bool TelemetryUnits::ValidateFinite(std::string_view label, double value) {
  if (!std::isfinite(value)) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                         "Telemetry %.*s contains non-finite value (%.6f)",
                         static_cast<int>(label.size()), label.data(), value);
    return false;
  }
  return true;
}

inline bool TelemetryUnits::ValidateMagnitude(std::string_view label, double value,
                                              double max_abs) {
  const double threshold = std::abs(max_abs);
  if (threshold < 0.0 || !std::isfinite(threshold)) {
    return true;
  }
  if (std::fabs(value) > threshold) {
    fsai::sim::log::Logf(
        fsai::sim::log::Level::kError,
        "Telemetry %.*s = %.6f exceeds ±%.6f expected range", static_cast<int>(label.size()),
        label.data(), value, threshold);
    return false;
  }
  return true;
}

inline bool TelemetryUnits::ValidateRadians(std::string_view label, double value,
                                            double max_abs) {
  if (!ValidateFinite(label, value)) {
    return false;
  }
  return ValidateMagnitude(label, value, max_abs);
}

inline bool TelemetryUnits::ValidateSteerAngle(std::string_view label, double value,
                                               double max_abs) {
  return ValidateRadians(label, value, max_abs);
}

inline bool TelemetryUnits::ValidateAngularRate(std::string_view label, double value,
                                                double max_abs) {
  if (!ValidateFinite(label, value)) {
    return false;
  }
  return ValidateMagnitude(label, value, max_abs);
}

inline double TelemetryUnits::RadToDeg(double rad, std::string_view label, double max_abs) {
  ValidateRadians(label, rad, max_abs);
  return rad * kRadToDeg;
}

inline double TelemetryUnits::DegToRad(double deg, std::string_view label, double max_abs) {
  if (!ValidateFinite(label, deg)) {
    return deg * kDegToRad;
  }
  ValidateMagnitude(label, deg, max_abs);
  return deg * kDegToRad;
}

inline Eigen::Vector3d TelemetryUnits::VeloxPlanarPosition(double x, double y) {
  return Eigen::Vector3d{x, y, 0.0};
}

inline Eigen::Vector3d TelemetryUnits::VeloxPlanarVelocity(double x, double y) {
  return Eigen::Vector3d{x, y, 0.0};
}

inline Eigen::Vector3d TelemetryUnits::BodyAccelerationToGlobal(double longitudinal,
                                                                double lateral,
                                                                double heading_rad) {
  ValidateRadians("telemetry.heading", heading_rad, kMaxTelemetryAngleRad * 2.0);
  const double cos_heading = std::cos(heading_rad);
  const double sin_heading = std::sin(heading_rad);
  const double ax_global = longitudinal * cos_heading - lateral * sin_heading;
  const double ay_global = longitudinal * sin_heading + lateral * cos_heading;
  return Eigen::Vector3d(ax_global, ay_global, 0.0);
}

inline Eigen::Vector3d TelemetryUnits::OpenGLGroundPosition(double velox_x, double velox_y) {
  // Axis mapping: Velox uses XY for ground while the renderer/world uses XZ so Y maps to Z.
  return Eigen::Vector3d(velox_x, 0.0, velox_y);
}

}  // namespace fsai::sim::app
