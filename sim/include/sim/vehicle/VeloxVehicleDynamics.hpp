#pragma once

#include <filesystem>
#include <vector>

#include "simulation/simulation_daemon.hpp"
#include "simulation/user_input.hpp"
#include <telemetry/telemetry.hpp>

#include "Transform.h"
#include "VehicleState.hpp"
#include "WheelsInfo.h"
#include "sim/architecture/IVehicleDynamics.hpp"

namespace fsai::vehicle {

class VeloxVehicleDynamics : public IVehicleDynamics {
public:
    struct Config {
        std::filesystem::path config_root{"velox/config"};
        std::filesystem::path parameter_root{"velox/parameters"};
        velox::simulation::ModelType model{velox::simulation::ModelType::MB};
        int vehicle_id{1};

        static Config FromVehicleConfig(const std::filesystem::path& vehicle_config,
                                        Config base = Config{});
    };

    VeloxVehicleDynamics();
    explicit VeloxVehicleDynamics(const Config& config);

    void set_command(float throttle, float brake, float steer) override;
    void step(double dt_seconds) override;

    void set_state(const VehicleState& state, const Transform& transform) override;
    void reset_input() override;

    const VehicleState& state() const override { return state_; }
    const Transform& transform() const override { return transform_; }
    const WheelsInfo& wheels_info() const override { return wheels_info_; }

    const velox::telemetry::SimulationTelemetry& telemetry() const { return telemetry_; }
    double wheel_radius() const { return wheel_radius_; }

private:
    void sync_from_telemetry();

    velox::simulation::SimulationDaemon daemon_;
    velox::simulation::UserInput current_input_{};
    velox::telemetry::SimulationTelemetry telemetry_{};

    VehicleState state_{};
    Transform transform_{};
    WheelsInfo wheels_info_{WheelsInfo_default()};

    double wheel_radius_{0.0};
    double timestamp_{0.0};

    float throttle_command_{0.0f};
    float brake_command_{0.0f};
    float steer_command_{0.0f};
};

}  // namespace fsai::vehicle

