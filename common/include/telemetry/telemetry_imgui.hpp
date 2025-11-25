#pragma once

#include "imgui.h"
#include "telemetry/telemetry.hpp"

namespace velox::telemetry {

inline void draw_telemetry_imgui(const SimulationTelemetry& telemetry, const char* label = "Telemetry")
{
    ImGui::Separator();
    ImGui::TextUnformatted(label);

    const ImVec4 latch_color = telemetry.low_speed_engaged ? ImVec4(1.0f, 0.62f, 0.26f, 1.0f)
                                                           : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
    const ImVec4 drift_color = telemetry.traction.drift_mode ? ImVec4(0.78f, 0.92f, 0.36f, 1.0f)
                                                             : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
    const ImVec4 detector_color = telemetry.detector_forced ? ImVec4(1.0f, 0.35f, 0.35f, 1.0f)
                                                            : latch_color;
    const char* stage_label = safety_stage_to_string(telemetry.safety_stage);
    ImGui::TextColored(latch_color,
                       "Low-speed safety latch: %s",
                       telemetry.low_speed_engaged ? "ENGAGED" : "Released");
    ImGui::TextColored(detector_color,
                       "Safety stage: %s (severity %.2f)%s",
                       stage_label,
                       telemetry.detector_severity,
                       telemetry.detector_forced ? " [detector]" : "");
    ImGui::TextColored(drift_color, "Drift mode: %s", telemetry.traction.drift_mode ? "ACTIVE" : "Off");

    ImGui::Text("Pose: (%.3f, %.3f) yaw %.3f rad",
                telemetry.pose.x,
                telemetry.pose.y,
                telemetry.pose.yaw);
    ImGui::Text("Velocity: speed %.3f m/s | v_long %.3f | v_lat %.3f | yaw_rate %.3f",
                telemetry.velocity.speed,
                telemetry.velocity.longitudinal,
                telemetry.velocity.lateral,
                telemetry.velocity.yaw_rate);
    ImGui::Text("Global velocity: (%.3f, %.3f) m/s",
                telemetry.velocity.global_x,
                telemetry.velocity.global_y);
    ImGui::Text("Acceleration: long %.3f m/s^2 | lat %.3f m/s^2",
                telemetry.acceleration.longitudinal,
                telemetry.acceleration.lateral);
    ImGui::Text("Slip angle: %.4f rad", telemetry.traction.slip_angle);
    ImGui::Text("Front slip: %.4f rad  Rear slip: %.4f rad",
                telemetry.traction.front_slip_angle,
                telemetry.traction.rear_slip_angle);
    ImGui::Text("Lateral utilisation: %.3f%s",
                telemetry.traction.lateral_force_saturation,
                telemetry.traction.drift_mode ? " (drift)" : "");

    ImGui::Separator();
    ImGui::Text("Steering: desired %.3f rad (rate %.3f)",
                telemetry.steering.desired_angle,
                telemetry.steering.desired_rate);
    ImGui::Text("Steering: actual  %.3f rad (rate %.3f)",
                telemetry.steering.actual_angle,
                telemetry.steering.actual_rate);

    ImGui::Separator();
    ImGui::Text("Controller accel: %.3f m/s^2  throttle %.2f  brake %.2f",
                telemetry.controller.acceleration,
                telemetry.controller.throttle,
                telemetry.controller.brake);
    ImGui::Text("Forces [N]: drive %.3f | brake %.3f | regen %.3f | hydraulic %.3f",
                telemetry.controller.drive_force,
                telemetry.controller.brake_force,
                telemetry.controller.regen_force,
                telemetry.controller.hydraulic_force);
    ImGui::Text("Drag %.3f  Rolling %.3f",
                telemetry.controller.drag_force,
                telemetry.controller.rolling_force);

    ImGui::Separator();
    ImGui::Text("Powertrain: drive %.3f Nm | regen %.3f Nm | total %.3f Nm",
                telemetry.powertrain.drive_torque,
                telemetry.powertrain.regen_torque,
                telemetry.powertrain.total_torque);
    ImGui::Text("Power [W]: mechanical %.3f | battery %.3f | SOC %.2f %%",
                telemetry.powertrain.mechanical_power,
                telemetry.powertrain.battery_power,
                telemetry.powertrain.soc * 100.0);

    auto draw_axle = [](const char* name, const AxleTelemetry& axle) {
        ImGui::Text("%s axle torque: drive %.3f Nm | brake %.3f Nm | regen %.3f Nm (normal %.1f N)",
                    name,
                    axle.drive_torque,
                    axle.brake_torque,
                    axle.regen_torque,
                    axle.normal_force);
        ImGui::Text("%s wheel L: speed %.3f m/s | slip %.3f | mu %.3f",
                    name,
                    axle.left.speed,
                    axle.left.slip_ratio,
                    axle.left.friction_utilization);
        ImGui::Text("%s wheel R: speed %.3f m/s | slip %.3f | mu %.3f",
                    name,
                    axle.right.speed,
                    axle.right.slip_ratio,
                    axle.right.friction_utilization);
    };

    draw_axle("Front", telemetry.front_axle);
    draw_axle("Rear", telemetry.rear_axle);

    ImGui::Separator();
    ImGui::Text("Totals: distance %.2f m | energy %.2f kJ",
                telemetry.totals.distance_traveled_m,
                telemetry.totals.energy_consumed_joules * 1e-3);
}

} // namespace velox::telemetry

