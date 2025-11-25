#include <iostream>
#include <vector>

#include "simulation/simulation_daemon.hpp"
#include "telemetry/telemetry.hpp"

int main()
{
    using namespace velox::simulation;

    SimulationDaemon::InitParams init{};
    init.model          = ModelType::ST;
    init.vehicle_id     = 1;
    init.config_root    = "config";
    init.parameter_root = "parameters";
    init.control_mode   = ControlMode::Direct;
    init.use_default_log_sink();

    SimulationDaemon daemon(init);

    ResetParams reset{};
    reset.control_mode  = ControlMode::Direct;
    reset.initial_state = std::vector<double>(7, 0.0);
    reset.dt            = 0.05;
    daemon.reset(reset);

    UserInput input{};
    input.control_mode   = ControlMode::Direct;
    input.timestamp      = 0.0;
    input.dt             = 0.05;
    input.steering_angle = daemon.steering_controller()->max_angle() * 0.25;
    input.axle_torques   = {150.0};

    for (int i = 0; i < 20; ++i) {
        auto telemetry = daemon.step(input);
        std::cout << "t=" << telemetry.totals.simulation_time_s
                  << "s accel=" << telemetry.controller.acceleration << " m/s^2"
                  << " steering=" << telemetry.steering.actual_angle << " rad"
                  << " torque=" << telemetry.powertrain.total_torque << " Nm" << std::endl;
        input.timestamp += input.dt;
    }

    return 0;
}
