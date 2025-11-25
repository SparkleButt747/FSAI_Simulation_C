#include "sim/vehicle/VeloxVehicleDynamics.hpp"

#include <algorithm>
#include <cmath>

#include "common/logging.hpp"

namespace fsai::vehicle {
namespace {
constexpr double kTwoPi = 2.0 * M_PI;
}

VeloxVehicleDynamics::VeloxVehicleDynamics(const Config& config)
    : daemon_([&config]() {
          velox::simulation::SimulationDaemon::InitParams init{};
          init.model = config.model;
          init.vehicle_id = config.vehicle_id;
          init.config_root = config.config_root;
          init.parameter_root = config.parameter_root;
          init.use_default_log_sink();
          return init;
      }()) {
    wheel_radius_ = daemon_.vehicle_parameters().R_w;
    current_input_.control_mode = velox::simulation::ControlMode::Keyboard;
    current_input_.steering_nudge = 0.0;
    current_input_.longitudinal = {};
    current_input_.timestamp = 0.0;
    current_input_.dt = 0.0;

    try {
        telemetry_ = daemon_.snapshot().telemetry;
    } catch (const std::exception& ex) {
        velox::logging::log_warning("VeloxVehicleDynamics", ex.what());
    }
    sync_from_telemetry();
}

void VeloxVehicleDynamics::set_command(float throttle, float brake, float steer) {
    throttle_command_ = std::clamp(throttle, 0.0f, 1.0f);
    brake_command_ = std::clamp(brake, 0.0f, 1.0f);
    steer_command_ = steer;
}

void VeloxVehicleDynamics::step(double dt_seconds) {
    timestamp_ += dt_seconds;

    current_input_.longitudinal.throttle = static_cast<double>(throttle_command_);
    current_input_.longitudinal.brake = static_cast<double>(brake_command_);
    current_input_.timestamp = timestamp_;
    current_input_.dt = dt_seconds;

    if (const auto* steering_wheel = daemon_.steering_wheel()) {
        const auto& wheel_cfg = steering_wheel->config();
        const double nudge_scale = (wheel_cfg.nudge_angle != 0.0) ? wheel_cfg.nudge_angle : 1.0;
        const double current_angle = steering_wheel->last_output().angle;
        const double desired_nudge = (steer_command_ - current_angle) / nudge_scale;
        current_input_.steering_nudge = std::clamp(desired_nudge, -1.0, 1.0);
    } else {
        current_input_.steering_nudge = steer_command_;
    }

    telemetry_ = daemon_.step(current_input_);
    sync_from_telemetry();
}

void VeloxVehicleDynamics::set_state(const VehicleState& state, const Transform& transform) {
    state_ = state;
    transform_ = transform;

    auto snapshot = daemon_.snapshot();
    auto reset_state = snapshot.state;

    if (reset_state.size() > 0) {
        reset_state[0] = state.position.x();
    }
    if (reset_state.size() > 1) {
        reset_state[1] = state.position.y();
    }
    if (reset_state.size() > 2) {
        reset_state[2] = transform.yaw;
    }
    if (reset_state.size() > 3) {
        const double heading = transform.yaw;
        const double v_long = std::cos(heading) * state.velocity.x() + std::sin(heading) * state.velocity.y();
        reset_state[3] = v_long;
    }
    if (reset_state.size() > 4) {
        reset_state[4] = state.yaw;
    }
    if (reset_state.size() > 5) {
        reset_state[5] = state.rotation.z();
    }
    if (reset_state.size() > 10) {
        const double heading = transform.yaw;
        const double v_lat = -std::sin(heading) * state.velocity.x() + std::cos(heading) * state.velocity.y();
        reset_state[10] = v_lat;
    }

    velox::simulation::ResetParams params{};
    params.initial_state = reset_state;
    params.dt = snapshot.dt;
    daemon_.reset(params);

    telemetry_ = daemon_.snapshot().telemetry;
    sync_from_telemetry();
}

void VeloxVehicleDynamics::reset_input() {
    throttle_command_ = 0.0f;
    brake_command_ = 0.0f;
    steer_command_ = 0.0f;
    current_input_.longitudinal = {};
}

void VeloxVehicleDynamics::sync_from_telemetry() {
    state_.position = Eigen::Vector3d(telemetry_.pose.x, telemetry_.pose.y, 0.0);
    state_.yaw = telemetry_.pose.yaw;

    const double heading = telemetry_.pose.yaw + telemetry_.traction.slip_angle;
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);

    state_.velocity = Eigen::Vector3d(telemetry_.velocity.global_x, telemetry_.velocity.global_y, 0.0);
    state_.rotation = Eigen::Vector3d(0.0, 0.0, telemetry_.velocity.yaw_rate);

    const double ax_body = telemetry_.acceleration.longitudinal;
    const double ay_body = telemetry_.acceleration.lateral;
    const double ax_global = ax_body * cos_heading - ay_body * sin_heading;
    const double ay_global = ax_body * sin_heading + ay_body * cos_heading;
    state_.acceleration = Eigen::Vector3d(ax_global, ay_global, 0.0);

    transform_.position.x = static_cast<float>(state_.position.x());
    transform_.position.z = static_cast<float>(state_.position.y());
    transform_.yaw = static_cast<float>(state_.yaw);

    const double safe_radius = std::max(wheel_radius_, 1e-6);
    const double front_left_rpm = (telemetry_.front_axle.left.speed / (kTwoPi * safe_radius)) * 60.0;
    const double front_right_rpm = (telemetry_.front_axle.right.speed / (kTwoPi * safe_radius)) * 60.0;
    const double rear_left_rpm = (telemetry_.rear_axle.left.speed / (kTwoPi * safe_radius)) * 60.0;
    const double rear_right_rpm = (telemetry_.rear_axle.right.speed / (kTwoPi * safe_radius)) * 60.0;

    wheels_info_.lf_speed = static_cast<float>(front_left_rpm);
    wheels_info_.rf_speed = static_cast<float>(front_right_rpm);
    wheels_info_.lb_speed = static_cast<float>(rear_left_rpm);
    wheels_info_.rb_speed = static_cast<float>(rear_right_rpm);
    wheels_info_.steering = static_cast<float>(telemetry_.steering.actual_angle);
}

}  // namespace fsai::vehicle

