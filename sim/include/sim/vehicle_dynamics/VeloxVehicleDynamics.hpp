#pragma once

#include <memory>
#include <vector>

#include "Transform.h"
#include "VehicleState.hpp"
#include "WheelsInfo.h"
#include "sim/architecture/IVehicleDynamics.hpp"
#include "simulation_daemon.hpp"
#include "user_input.hpp"
#include "telemetry.hpp"

class VeloxVehicleDynamics : public fsai::vehicle::IVehicleDynamics {
public:
    VeloxVehicleDynamics();
    explicit VeloxVehicleDynamics(std::unique_ptr<velox::simulation::SimulationDaemon> daemon);

    void set_command(float throttle, float brake, float steer) override;
    void step(double dt_seconds) override;
    void set_state(const VehicleState& state, const Transform& transform) override;
    void reset_input() override;

    [[nodiscard]] const VehicleState& state() const override { return state_; }
    [[nodiscard]] const Transform& transform() const override { return transform_; }
    [[nodiscard]] const WheelsInfo& wheels_info() const override { return wheels_info_; }
    [[nodiscard]] const velox::telemetry::SimulationTelemetry& telemetry() const { return last_telemetry_; }

private:
    void refresh_input_limits();
    void update_cached_state(const velox::telemetry::SimulationTelemetry& telemetry);
    static std::unique_ptr<velox::simulation::SimulationDaemon> make_default_daemon();

    std::unique_ptr<velox::simulation::SimulationDaemon> daemon_{};
    velox::simulation::UserInput                         current_input_{};
    velox::simulation::UserInputLimits                   input_limits_{};
    velox::telemetry::SimulationTelemetry                last_telemetry_{};

    VehicleState state_{};
    Transform    transform_{};
    WheelsInfo   wheels_info_{WheelsInfo_default()};
};

