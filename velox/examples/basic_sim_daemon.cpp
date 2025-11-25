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
    init.use_default_log_sink();

    SimulationDaemon daemon(init);

    UserInput input{};
    input.longitudinal.throttle     = 0.25;
    input.longitudinal.brake        = 0.0;
    input.steering_nudge            = 0.0;
    input.dt                        = 0.2; // intentionally large to trigger sub-stepping warnings
    input.timestamp                 = 0.0;

    for (int i = 0; i < 100; ++i) {
        auto telemetry = daemon.step(input);
        std::cout << "t=" << telemetry.totals.simulation_time_s
                  << "s  pose=(" << telemetry.pose.x << ", " << telemetry.pose.y << ")"
                  << "  speed=" << telemetry.velocity.speed << " m/s" << std::endl;
        input.timestamp += input.dt;
    }

    auto final_snapshot = daemon.snapshot();
    std::cout << "Final timestep: " << final_snapshot.dt << " s, state dimension: "
              << final_snapshot.state.size() << std::endl;

    return 0;
}
