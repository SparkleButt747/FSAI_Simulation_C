#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "simulation/simulation_daemon.hpp"
#include "simulation/user_input.hpp"
#include <telemetry/telemetry.hpp>

#include "Transform.h"
#include "VehicleState.hpp"
#include "WheelsInfo.h"
#include "sim/architecture/IVehicleDynamics.hpp"
#include "adsdv_dbc.hpp"

namespace fsai::vehicle {

class VeloxVehicleDynamics : public IVehicleDynamics {
public:
    struct Command {
        float throttle{0.0f};  // normalized [0,1]
        float brake{0.0f};     // normalized [0,1]
        float steer_rad{0.0f};
        std::optional<float> front_axle_torque_nm{};
        std::optional<float> rear_axle_torque_nm{};
    };

    struct Config {
        std::filesystem::path config_root{"velox/config"};
        std::filesystem::path parameter_root{"velox/parameters"};
        velox::simulation::ModelType model{velox::simulation::ModelType::MB};
        int vehicle_id{1};
        double max_steer_rad{fsai::sim::svcu::dbc::kMaxSteerDeg * fsai::sim::svcu::dbc::kDegToRad};
        float front_axle_max_torque_nm{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};
        float rear_axle_max_torque_nm{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};

        static Config FromVehicleConfig(const std::filesystem::path& vehicle_config,
                                        Config base);
    };

    VeloxVehicleDynamics();
    explicit VeloxVehicleDynamics(const Config& config);

    void set_command(float throttle, float brake, float steer) override;
    void set_command(const Command& cmd);
    void step(double dt_seconds) override;

    void set_state(const VehicleState& state, const Transform& transform) override;
    void reset_input() override;

    const VehicleState& state() const override { return state_; }
    const Transform& transform() const override { return transform_; }
    const WheelsInfo& wheels_info() const override { return wheels_info_; }

    const velox::telemetry::SimulationTelemetry& telemetry() const { return telemetry_; }
    double wheel_radius() const { return wheel_radius_; }
    bool healthy() const { return healthy_; }
    const std::string& last_error() const { return last_error_; }

private:
    void sync_from_telemetry();
    double steer_limit_rad() const;
    void log_if_clamped(const Command& requested, const Command& applied) const;
    bool validate_telemetry(const velox::telemetry::SimulationTelemetry& telemetry) const;

    velox::simulation::SimulationDaemon daemon_;
    velox::simulation::UserInput current_input_{};
    velox::telemetry::SimulationTelemetry telemetry_{};

    VehicleState state_{};
    Transform transform_{};
    WheelsInfo wheels_info_{WheelsInfo_default()};

    double max_steer_config_rad_{fsai::sim::svcu::dbc::kMaxSteerDeg * fsai::sim::svcu::dbc::kDegToRad};
    float front_axle_max_torque_nm_{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};
    float rear_axle_max_torque_nm_{fsai::sim::svcu::dbc::kMaxAxleTorqueNm};
    double wheel_radius_{0.0};
    double timestamp_{0.0};

    float throttle_command_{0.0f};
    float brake_command_{0.0f};
    float steer_command_{0.0f};
    bool healthy_{true};
    std::string last_error_;
};

}  // namespace fsai::vehicle
