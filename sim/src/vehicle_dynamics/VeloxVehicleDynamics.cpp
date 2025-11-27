#include "sim/vehicle/VeloxVehicleDynamics.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include "../../include/logging.hpp"
#include <yaml-cpp/yaml.h>

#include "sim/app/telemetry_units.hpp"

namespace fsai::vehicle {
using fsai::sim::app::TelemetryUnits;
namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr float kCmdEpsilon = 1e-5f;

velox::simulation::ModelType ParseModelType(std::string_view name)
{
    std::string normalized{name};
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized == "mb" || normalized == "multi-body" || normalized == "multibody") {
        return velox::simulation::ModelType::MB;
    }
    if (normalized == "st" || normalized == "single-track" || normalized == "singletrack") {
        return velox::simulation::ModelType::ST;
    }
    if (normalized == "std" || normalized == "kinematic-st" || normalized == "kinematic") {
        return velox::simulation::ModelType::STD;
    }

    throw std::invalid_argument("Unsupported Velox model type: " + normalized);
}
}

VeloxVehicleDynamics::Config VeloxVehicleDynamics::Config::FromVehicleConfig(
    const std::filesystem::path& vehicle_config,
    Config base)
{
    if (vehicle_config.empty()) {
        return base;
    }

    try {
        const YAML::Node config = YAML::LoadFile(vehicle_config.string());
        const YAML::Node velox = config["velox"];
        if (!velox) {
            return base;
        }

        if (const auto id = velox["vehicle_id"]) {
            base.vehicle_id = id.as<int>();
        }
        if (const auto model = velox["model"]) {
            base.model = ParseModelType(model.as<std::string>());
        }
    } catch (const std::exception& ex) {
        fsai::sim::log::LogWarning(ex.what());
    }

    return base;
}

VeloxVehicleDynamics::VeloxVehicleDynamics()
    : VeloxVehicleDynamics(Config{})
{
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
    max_steer_config_rad_ = std::max(0.0, config.max_steer_rad);
    front_axle_max_torque_nm_ =
        std::clamp(config.front_axle_max_torque_nm, 0.0f,
                   fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
    rear_axle_max_torque_nm_ =
        std::clamp(config.rear_axle_max_torque_nm, 0.0f,
                   fsai::sim::svcu::dbc::kMaxAxleTorqueNm);
    wheel_radius_ = daemon_.vehicle_parameters().R_w;
    current_input_.control_mode = velox::simulation::ControlMode::Keyboard;
    current_input_.steering_nudge = 0.0;
    current_input_.longitudinal = {};
    current_input_.timestamp = 0.0;
    current_input_.dt = 0.0;

    try {
        telemetry_ = daemon_.snapshot().telemetry;
    } catch (const std::exception& ex) {
        fsai::sim::log::LogWarning(ex.what());
    }
    sync_from_telemetry();
}

void VeloxVehicleDynamics::set_command(float throttle, float brake, float steer) {
    Command cmd{};
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.steer_rad = steer;
    set_command(cmd);
}

void VeloxVehicleDynamics::set_command(const Command& cmd) {
    healthy_ = true;
    last_error_.clear();

    auto finite_or_zero = [&](float value, const char* field) {
        if (!std::isfinite(value)) {
            fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                                 "Velox command %s is not finite (%.4f); zeroing.",
                                 field, value);
            healthy_ = false;
            last_error_ = "Non-finite command input";
            return 0.0f;
        }
        return value;
    };

    Command applied = cmd;

    applied.throttle = finite_or_zero(cmd.throttle, "throttle");
    applied.brake = finite_or_zero(cmd.brake, "brake");
    applied.steer_rad = finite_or_zero(cmd.steer_rad, "steer_rad");

    const bool bypass = true;

    if (!bypass) {
        // In bypass mode, we skip torque mode processing.
        applied.throttle = std::clamp(applied.throttle, 0.0f, 1.0f);
        applied.brake = std::clamp(applied.brake, 0.0f, 1.0f);

    const bool torque_mode =
        (cmd.front_axle_torque_nm.has_value() || cmd.rear_axle_torque_nm.has_value()) &&
        std::abs(cmd.throttle) <= kCmdEpsilon && std::abs(cmd.brake) <= kCmdEpsilon;
    if (torque_mode) {
        const float front_raw = finite_or_zero(cmd.front_axle_torque_nm.value_or(0.0f), "front_axle_torque_nm");
        const float rear_raw = finite_or_zero(cmd.rear_axle_torque_nm.value_or(0.0f), "rear_axle_torque_nm");
        const float front_clamped = std::clamp(front_raw, -front_axle_max_torque_nm_, front_axle_max_torque_nm_);
        const float rear_clamped = std::clamp(rear_raw, -rear_axle_max_torque_nm_, rear_axle_max_torque_nm_);

        const float max_drive_total = std::max(front_axle_max_torque_nm_, 0.0f) +
                                      std::max(rear_axle_max_torque_nm_, 0.0f);
        if (max_drive_total <= std::numeric_limits<float>::epsilon()) {
            fsai::sim::log::LogError("Velox command rejected: axle torque maxima are zero");
            healthy_ = false;
            last_error_ = "No torque limits configured";
            applied.throttle = 0.0f;
            applied.brake = 1.0f;
        } else {
            const double drive_torque = std::max<double>(0.0, static_cast<double>(front_clamped)) +
                                        std::max<double>(0.0, static_cast<double>(rear_clamped));
            const double brake_torque = std::max<double>(0.0, -static_cast<double>(front_clamped)) +
                                        std::max<double>(0.0, -static_cast<double>(rear_clamped));
            applied.throttle = static_cast<float>(drive_torque / max_drive_total);
            applied.brake = static_cast<float>(brake_torque / max_drive_total);
        }
    }

    const double steer_limit = steer_limit_rad();
    if (steer_limit > 0.0) {
        if (applied.steer_rad < -steer_limit || applied.steer_rad > steer_limit) {
            fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                                 "Steering request %.3f rad clamped to ±%.3f rad",
                                 applied.steer_rad, steer_limit);
        }
        applied.steer_rad = static_cast<float>(
            std::clamp(static_cast<double>(applied.steer_rad), -steer_limit, steer_limit));
    }

    const Command requested = applied;
    applied.throttle = std::clamp(applied.throttle, 0.0f, 1.0f);
    applied.brake = std::clamp(applied.brake, 0.0f, 1.0f);
    if (applied.throttle + applied.brake > 1.0f) {
        const float net = applied.throttle - applied.brake;
        applied.throttle = std::clamp(net, 0.0f, 1.0f);
        applied.brake = std::clamp(-net, 0.0f, 1.0f);
        fsai::sim::log::LogWarning("Throttle/brake both requested; projecting onto single axis.");
    }

        log_if_clamped(requested, applied);

    }

    applied.steer_rad = applied.steer_rad / 0.366519f;  // No clamping needed here; done above.

    std::printf("Received Velox Command - Throttle: %f, Brake: %f, Steer(rad): %f\n",
            applied.throttle,
            applied.brake,
            applied.steer_rad);

    throttle_command_ = applied.throttle;
    brake_command_ = applied.brake;  // temp fix Scale brake to match expected behavior.
    steer_command_ = applied.steer_rad;
}

void VeloxVehicleDynamics::step(double dt_seconds) {
    const double previous_timestamp = timestamp_;
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

    try {
        const auto telemetry = daemon_.step(current_input_);
        if (!validate_telemetry(telemetry)) {
            healthy_ = false;
            last_error_ = "Velox telemetry validation failed";
            timestamp_ = previous_timestamp;
            return;
        }
        telemetry_ = telemetry;
        std::printf("Velox Telemetry - Speed: %.3f m/s, Position: (%.3f, %.3f), Yaw: %.3f rad\n",
                    telemetry_.velocity.speed,
                    telemetry_.pose.x,
                    telemetry_.pose.y,
                    telemetry_.pose.yaw);


        healthy_ = true;
        last_error_.clear();
        sync_from_telemetry();
    } catch (const std::exception& ex) {
        healthy_ = false;
        last_error_ = ex.what();
        timestamp_ = previous_timestamp;
        fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                             "Velox step failed: %s", ex.what());
    }
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
    // Keep the state in Velox XY coordinates so downstream systems stay aligned with telemetry.
    state_.position = TelemetryUnits::VeloxPlanarPosition(telemetry_.pose.x, telemetry_.pose.y);
    state_.yaw = telemetry_.pose.yaw;

    const double heading = state_.yaw + telemetry_.traction.slip_angle;
    TelemetryUnits::ValidateRadians("telemetry.heading (yaw + slip)", heading,
                                    TelemetryUnits::kMaxTelemetryAngleRad * 2.0);

    // Velox global velocity reports planar XY axes, so we reuse those directly.
    state_.velocity =
        TelemetryUnits::VeloxPlanarVelocity(telemetry_.velocity.global_x,
                                            telemetry_.velocity.global_y);

    const double yaw_rate = telemetry_.velocity.yaw_rate;
    TelemetryUnits::ValidateAngularRate("telemetry.velocity.yaw_rate", yaw_rate);
    state_.rotation = Eigen::Vector3d(0.0, 0.0, yaw_rate);

    // Convert body-frame acceleration (longitudinal/lateral) into the world frame.
    state_.acceleration = TelemetryUnits::BodyAccelerationToGlobal(
        telemetry_.acceleration.longitudinal, telemetry_.acceleration.lateral, heading);

    // Velox XY → OpenGL XZ axis mapping for the renderer/world transform.
    const Eigen::Vector3d world_position =
        TelemetryUnits::OpenGLGroundPosition(telemetry_.pose.x, telemetry_.pose.y);
    transform_.position.x = static_cast<float>(world_position.x());
    transform_.position.y = static_cast<float>(world_position.y());
    transform_.position.z = static_cast<float>(world_position.z());
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

double VeloxVehicleDynamics::steer_limit_rad() const {
    if (const auto* final = daemon_.steering_controller()) {
        return final->config().max_angle;
    }
    if (const auto* wheel = daemon_.steering_wheel()) {
        return wheel->config().max_angle;
    }
    return max_steer_config_rad_;
}

void VeloxVehicleDynamics::log_if_clamped(const Command& requested, const Command& applied) const {
    const auto changed = [](float a, float b) { return std::abs(a - b) > 1e-6f; };
    if (changed(requested.throttle, applied.throttle)) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                             "Throttle clamped from %.3f to %.3f", requested.throttle, applied.throttle);
    }
    if (changed(requested.brake, applied.brake)) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                             "Brake clamped from %.3f to %.3f", requested.brake, applied.brake);
    }
    if (changed(requested.steer_rad, applied.steer_rad)) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                             "Steering clamped from %.3f rad to %.3f rad", requested.steer_rad, applied.steer_rad);
    }
}

bool VeloxVehicleDynamics::validate_telemetry(
    const velox::telemetry::SimulationTelemetry& telemetry) const {
    if (!(TelemetryUnits::ValidateFinite("telemetry.pose.x", telemetry.pose.x) &&
          TelemetryUnits::ValidateFinite("telemetry.pose.y", telemetry.pose.y) &&
          TelemetryUnits::ValidateRadians("telemetry.pose.yaw", telemetry.pose.yaw))) {
        return false;
    }
    if (!TelemetryUnits::ValidateRadians("telemetry.traction.slip_angle",
                                         telemetry.traction.slip_angle)) {
        return false;
    }
    if (!(TelemetryUnits::ValidateFinite("telemetry.velocity.global_x",
                                         telemetry.velocity.global_x) &&
          TelemetryUnits::ValidateFinite("telemetry.velocity.global_y",
                                         telemetry.velocity.global_y) &&
          TelemetryUnits::ValidateAngularRate("telemetry.velocity.yaw_rate",
                                              telemetry.velocity.yaw_rate))) {
        return false;
    }
    if (!(TelemetryUnits::ValidateFinite("telemetry.acceleration.longitudinal",
                                         telemetry.acceleration.longitudinal) &&
          TelemetryUnits::ValidateFinite("telemetry.acceleration.lateral",
                                         telemetry.acceleration.lateral))) {
        return false;
    }
    if (!TelemetryUnits::ValidateSteerAngle("telemetry.steering.actual_angle",
                                            telemetry.steering.actual_angle)) {
        return false;
    }
    if (!(wheel_radius_ > 0.0) || !std::isfinite(wheel_radius_)) {
        fsai::sim::log::LogError("Velox wheel radius must be positive/finite for telemetry conversion");
        return false;
    }
    return true;
}

}  // namespace fsai::vehicle
