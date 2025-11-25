#include "sim/vehicle_dynamics/VeloxVehicleDynamics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "common/errors.hpp"
#include "controllers/longitudinal/powertrain.hpp"

using velox::controllers::longitudinal::PowertrainConfig;
using velox::simulation::ControlMode;
using velox::simulation::SimulationDaemon;
using velox::simulation::UserInput;
using velox::simulation::UserInputLimits;
using velox::telemetry::SimulationTelemetry;

namespace {
constexpr double kDefaultDtSeconds = 0.01;

double clamp01(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

std::vector<double> make_reset_state(const VehicleState& state,
                                     const Transform& transform,
                                     std::size_t dimension)
{
    std::vector<double> reset_state(dimension, 0.0);
    if (!reset_state.empty()) {
        reset_state[0] = state.position.x();
    }
    if (reset_state.size() > 1) {
        reset_state[1] = state.position.y();
    }
    if (reset_state.size() > 2) {
        reset_state[2] = transform.yaw;
    }
    if (reset_state.size() > 3) {
        reset_state[3] = state.velocity.head<2>().norm();
    }
    if (reset_state.size() > 4) {
        reset_state[4] = transform.yaw;
    }
    if (reset_state.size() > 5) {
        reset_state[5] = state.rotation.z();
    }
    return reset_state;
}

} // namespace

VeloxVehicleDynamics::VeloxVehicleDynamics()
    : VeloxVehicleDynamics(make_default_daemon())
{
}

VeloxVehicleDynamics::VeloxVehicleDynamics(std::unique_ptr<SimulationDaemon> daemon)
    : daemon_(std::move(daemon))
{
    reset_input();
    refresh_input_limits();
}

void VeloxVehicleDynamics::set_command(float throttle, float brake, float steer)
{
    current_input_.control_mode          = ControlMode::Direct;
    current_input_.longitudinal.throttle = clamp01(static_cast<double>(throttle));
    current_input_.longitudinal.brake    = clamp01(static_cast<double>(brake));
    current_input_.steering_angle        = static_cast<double>(steer);

    double net_torque = current_input_.longitudinal.throttle - current_input_.longitudinal.brake;
    const auto& params = daemon_->vehicle_parameters();

    const double front_split = std::clamp(params.T_se, 0.0, 1.0);
    const double rear_split  = std::clamp(1.0 - front_split, 0.0, 1.0);

    const PowertrainConfig* powertrain_cfg = nullptr;
    if (const auto* accel = daemon_->accel_controller()) {
        powertrain_cfg = &accel->powertrain().config();
    }

    if (powertrain_cfg) {
        const double drive_torque = current_input_.longitudinal.throttle * powertrain_cfg->max_drive_torque;
        const double brake_torque = current_input_.longitudinal.brake * powertrain_cfg->max_regen_torque;
        net_torque = drive_torque - brake_torque;
    }

    current_input_.axle_torques.clear();
    if (front_split > 0.0) {
        current_input_.axle_torques.push_back(net_torque * front_split);
    }
    if (rear_split > 0.0) {
        current_input_.axle_torques.push_back(net_torque * rear_split);
    }
}

void VeloxVehicleDynamics::step(double dt_seconds)
{
    if (!daemon_) {
        throw ::velox::errors::SimulationError(VELOX_LOC("VeloxVehicleDynamics missing SimulationDaemon"));
    }

    current_input_.dt        = dt_seconds > 0.0 ? dt_seconds : kDefaultDtSeconds;
    current_input_.timestamp = last_telemetry_.totals.simulation_time_s + current_input_.dt;

    const UserInput sanitized = current_input_.clamped(input_limits_);
    const SimulationTelemetry telemetry = daemon_->step(sanitized);
    last_telemetry_ = telemetry;

    if (const auto* simulator = daemon_->simulator()) {
        state_.timestampNs = static_cast<uint64_t>(current_input_.timestamp * 1e9);
        const auto& sim_state = simulator->state();
        if (!sim_state.empty()) {
            // Keep a minimal alignment between cached pose and simulator state.
            if (sim_state.size() > 0) { state_.position.x() = sim_state[0]; }
            if (sim_state.size() > 1) { state_.position.y() = sim_state[1]; }
            if (sim_state.size() > 4) { state_.yaw = sim_state[4]; }
        }
    }

    update_cached_state(telemetry);
}

void VeloxVehicleDynamics::set_state(const VehicleState& state, const Transform& transform)
{
    state_     = state;
    transform_ = transform;

    if (daemon_ && daemon_->simulator()) {
        const auto current_state = daemon_->simulator()->state();
        const auto reset_state   = make_reset_state(state, transform, current_state.size());

        velox::simulation::ResetParams reset_params{};
        reset_params.initial_state = reset_state;
        reset_params.dt            = daemon_->simulator()->dt();
        reset_params.control_mode  = ControlMode::Direct;
        daemon_->reset(reset_params);
    }
}

void VeloxVehicleDynamics::reset_input()
{
    current_input_ = {};
    current_input_.control_mode = ControlMode::Direct;
    current_input_.axle_torques.clear();
}

void VeloxVehicleDynamics::refresh_input_limits()
{
    if (!daemon_) {
        input_limits_ = velox::simulation::kDefaultUserInputLimits;
        return;
    }

    const auto* steering_wheel = daemon_->steering_wheel();
    const auto* final_steer    = daemon_->steering_controller();
    const auto* accel          = daemon_->accel_controller();
    const PowertrainConfig* powertrain_cfg = accel ? &accel->powertrain().config() : nullptr;

    input_limits_ = UserInputLimits::from_vehicle(steering_wheel,
                                                  final_steer,
                                                  daemon_->vehicle_parameters(),
                                                  powertrain_cfg);
}

void VeloxVehicleDynamics::update_cached_state(const SimulationTelemetry& telemetry)
{
    state_.position.x() = telemetry.pose.x;
    state_.position.y() = telemetry.pose.y;
    state_.position.z() = 0.0;

    state_.velocity.x() = telemetry.velocity.global_x;
    state_.velocity.y() = telemetry.velocity.global_y;
    state_.velocity.z() = 0.0;

    state_.rotation << 0.0, 0.0, telemetry.velocity.yaw_rate;
    state_.acceleration << telemetry.acceleration.longitudinal,
        telemetry.acceleration.lateral,
        0.0;
    state_.yaw = telemetry.pose.yaw;

    transform_.position.x = static_cast<float>(telemetry.pose.x);
    transform_.position.y = 0.0f;
    transform_.position.z = static_cast<float>(telemetry.pose.y);
    transform_.yaw        = static_cast<float>(telemetry.pose.yaw);

    wheels_info_.lf_speed = static_cast<float>(telemetry.front_axle.left.speed);
    wheels_info_.rf_speed = static_cast<float>(telemetry.front_axle.right.speed);
    wheels_info_.lb_speed = static_cast<float>(telemetry.rear_axle.left.speed);
    wheels_info_.rb_speed = static_cast<float>(telemetry.rear_axle.right.speed);
    wheels_info_.steering = static_cast<float>(telemetry.steering.actual_angle);
}

std::unique_ptr<SimulationDaemon> VeloxVehicleDynamics::make_default_daemon()
{
    SimulationDaemon::InitParams init{};
    init.model          = velox::simulation::ModelType::ST;
    init.vehicle_id     = 1;
    init.config_root    = "velox/config";
    init.parameter_root = "velox/parameters";
    init.control_mode   = ControlMode::Direct;
    init.use_default_log_sink();
    return std::make_unique<SimulationDaemon>(init);
}

