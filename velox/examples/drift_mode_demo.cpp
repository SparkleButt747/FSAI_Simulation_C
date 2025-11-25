#include <algorithm>
#include <cmath>
#include <iostream>
#include <string_view>

#include "simulation/simulation_daemon.hpp"
#include "telemetry/telemetry.hpp"

namespace vsim = velox::simulation;

struct RunSummary {
    double max_yaw_rate{0.0};
    double max_slip_angle{0.0};
    double max_speed{0.0};
    double max_detector_severity{0.0};
    int    emergency_frames{0};
    bool   drift_mode{false};
    bool   low_speed_latched{false};
};

RunSummary run_oversteer_pass(vsim::SimulationDaemon& daemon, bool drift_enabled, std::string_view label)
{
    vsim::ResetParams reset{};
    reset.dt            = 0.02;
    reset.drift_enabled = drift_enabled;
    daemon.reset(reset);

    RunSummary summary{};
    summary.drift_mode = drift_enabled;

    double timestamp = 0.0;
    for (int i = 0; i < 300; ++i) {
        vsim::UserInput input{};
        input.timestamp             = timestamp;
        input.dt                    = 0.02;
        input.longitudinal.throttle = (i < 200) ? 0.25 : 0.05;
        input.steering_nudge        = 1.0;

        const auto telemetry = daemon.step(input);
        summary.max_yaw_rate  = std::max(summary.max_yaw_rate, std::abs(telemetry.velocity.yaw_rate));
        summary.max_slip_angle = std::max(summary.max_slip_angle, std::abs(telemetry.traction.slip_angle));
        summary.max_speed     = std::max(summary.max_speed, telemetry.velocity.speed);
        summary.max_detector_severity = std::max(summary.max_detector_severity, telemetry.detector_severity);
        summary.emergency_frames += telemetry.safety_stage == vsim::SafetyStage::Emergency ? 1 : 0;
        summary.low_speed_latched = summary.low_speed_latched || telemetry.low_speed_engaged;
        timestamp += input.dt;
    }

    const auto& telemetry = daemon.telemetry();
    std::cout << label << " drift=" << (telemetry.traction.drift_mode ? "on" : "off")
              << " max_speed=" << summary.max_speed
              << " max_yaw=" << summary.max_yaw_rate
              << " max_slip=" << summary.max_slip_angle
              << " max_severity=" << summary.max_detector_severity
              << " emergency_frames=" << summary.emergency_frames
              << " latch=" << (summary.low_speed_latched ? "engaged" : "released") << '\n';

    return summary;
}

void brake_and_steer(vsim::SimulationDaemon& daemon, bool drift_enabled)
{
    vsim::ResetParams reset{};
    reset.dt            = 0.02;
    reset.drift_enabled = drift_enabled;
    daemon.reset(reset);

    double timestamp = 0.0;
    std::cout << "[brake-and-steer] drift=" << (drift_enabled ? "on" : "off") << '\n';

    for (int i = 0; i < 180; ++i) {
        vsim::UserInput input{};
        input.timestamp = timestamp;
        input.dt        = 0.02;

        // Launch forward, then brake and saw the wheel to provoke the detector.
        if (i < 60) {
            input.longitudinal.throttle = 0.35;
            input.steering_nudge        = 0.15;
        } else {
            input.longitudinal.brake = 0.7;
            input.steering_nudge     = (i % 20 < 10) ? 0.9 : -0.9;
        }

        const auto telemetry = daemon.step(input);

        if (i % 15 == 0 || telemetry.detector_severity > 0.25) {
            std::cout << "  t=" << timestamp
                      << "s v=" << telemetry.velocity.speed
                      << " yaw=" << telemetry.velocity.yaw_rate
                      << " slip=" << telemetry.traction.slip_angle
                      << " severity=" << telemetry.detector_severity
                      << " stage=" << velox::telemetry::safety_stage_to_string(telemetry.safety_stage)
                      << '\n';
        }

        timestamp += input.dt;
    }
}

int main()
{
    vsim::SimulationDaemon::InitParams init{};
    init.model      = vsim::ModelType::STD;
    init.vehicle_id = 1;

    vsim::SimulationDaemon daemon(init);

    const auto safety_run = run_oversteer_pass(daemon, false, "[low-speed safety]");
    const auto drift_run  = run_oversteer_pass(daemon, true, "[drift profile]");

    brake_and_steer(daemon, false);
    brake_and_steer(daemon, true);

    if (drift_run.max_yaw_rate <= safety_run.max_yaw_rate ||
        drift_run.max_slip_angle <= safety_run.max_slip_angle) {
        std::cerr << "Drift mode did not increase yaw/slip allowance" << std::endl;
        return 1;
    }

    std::cout << "Drift mode demo complete: increased yaw/slip allowances confirmed." << std::endl;
    return 0;
}
