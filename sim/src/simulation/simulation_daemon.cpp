#include "simulation/simulation_daemon.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

#include "common/errors.hpp"

#include "models/vehiclemodels/init_mb.hpp"
#include "models/vehiclemodels/init_st.hpp"
#include "models/vehiclemodels/init_std.hpp"
#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"
#include "models/vehiclemodels/vehicle_dynamics_st.hpp"
#include "models/vehiclemodels/vehicle_dynamics_std.hpp"
#include "vehicle_parameters.hpp"

namespace velox::simulation {

UserInputLimits UserInputLimits::from_vehicle(const controllers::SteeringWheel* steering_wheel,
                                              const controllers::FinalSteerController* final_steer,
                                              const models::VehicleParameters& params,
                                              const controllers::longitudinal::PowertrainConfig* powertrain_cfg)
{
    UserInputLimits limits{};
    if (final_steer) {
        limits.min_steering_angle = final_steer->min_angle();
        limits.max_steering_angle = final_steer->max_angle();
    } else if (steering_wheel) {
        limits.min_steering_angle = -steering_wheel->config().max_angle;
        limits.max_steering_angle = steering_wheel->config().max_angle;
    } else {
        limits.min_steering_angle = params.steering.min;
        limits.max_steering_angle = params.steering.max;
    }

    const double max_drive_torque = powertrain_cfg ? powertrain_cfg->max_drive_torque : 0.0;
    const double max_regen_torque = powertrain_cfg ? powertrain_cfg->max_regen_torque : 0.0;

    const double front_split = params.T_se;
    const double rear_split  = 1.0 - front_split;

    if (front_split > 0.0) {
        limits.min_axle_torque.push_back(-max_regen_torque);
        limits.max_axle_torque.push_back(max_drive_torque);
    }
    if (rear_split > 0.0) {
        limits.min_axle_torque.push_back(-max_regen_torque);
        limits.max_axle_torque.push_back(max_drive_torque);
    }

    return limits;
}

UserInput UserInput::clamped(const UserInputLimits& limits) const
{
    validate(limits);
    UserInput copy = *this;
    if (control_mode == ControlMode::Keyboard) {
        copy.longitudinal.throttle =
            std::clamp(copy.longitudinal.throttle, limits.min_throttle, limits.max_throttle);
        copy.longitudinal.brake = std::clamp(copy.longitudinal.brake, limits.min_brake, limits.max_brake);
        copy.steering_nudge     = std::clamp(copy.steering_nudge, limits.min_steering_nudge, limits.max_steering_nudge);
    } else {
        copy.steering_angle = std::clamp(copy.steering_angle, limits.min_steering_angle, limits.max_steering_angle);

        if (copy.axle_torques.size() == limits.min_axle_torque.size() &&
            limits.min_axle_torque.size() == limits.max_axle_torque.size()) {
            for (std::size_t i = 0; i < copy.axle_torques.size(); ++i) {
                copy.axle_torques[i] = std::clamp(copy.axle_torques[i], limits.min_axle_torque[i], limits.max_axle_torque[i]);
            }
        }
    }
    if (copy.drift_toggle.has_value()) {
        copy.drift_toggle = std::clamp(*copy.drift_toggle, limits.min_drift_toggle, limits.max_drift_toggle);
    }
    return copy;
}

void UserInput::validate(const UserInputLimits& limits) const
{
    const auto require_finite = [](double value, const char* field) {
        if (!std::isfinite(value)) {
            std::ostringstream oss;
            oss << "UserInput." << field << " must be finite; got " << value;
            throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
        }
    };

    const auto require_in_range = [](double value, const char* field, double min, double max) {
        if (value < min || value > max) {
            std::ostringstream oss;
            oss << "UserInput." << field << " of " << value << " outside [" << min << ", " << max << "]";
            throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
        }
    };

    require_finite(timestamp, "timestamp");
    if (timestamp < 0.0) {
        std::ostringstream oss;
        oss << "UserInput.timestamp must be non-negative; got " << timestamp;
        throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
    }

    require_finite(dt, "dt");
    if (dt <= 0.0) {
        std::ostringstream oss;
        oss << "UserInput.dt must be positive; got " << dt;
        throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
    }

    switch (control_mode) {
        case ControlMode::Keyboard:
            require_finite(longitudinal.throttle, "longitudinal.throttle");
            require_in_range(longitudinal.throttle,
                             "longitudinal.throttle",
                             limits.min_throttle,
                             limits.max_throttle);

            require_finite(longitudinal.brake, "longitudinal.brake");
            require_in_range(longitudinal.brake, "longitudinal.brake", limits.min_brake, limits.max_brake);

            require_finite(steering_nudge, "steering_nudge");
            require_in_range(steering_nudge,
                             "steering_nudge",
                             limits.min_steering_nudge,
                             limits.max_steering_nudge);
            break;
        case ControlMode::Direct: {
            require_finite(steering_angle, "steering_angle");
            require_in_range(steering_angle, "steering_angle", limits.min_steering_angle, limits.max_steering_angle);

            const bool enforce_torque_limits = !limits.min_axle_torque.empty() || !limits.max_axle_torque.empty() ||
                                               !axle_torques.empty();
            if (enforce_torque_limits) {
                if (limits.min_axle_torque.size() != limits.max_axle_torque.size()) {
                    throw ::velox::errors::InputError(
                        VELOX_LOC("UserInputLimits torque bounds must be sized consistently"));
                }
                if (axle_torques.size() != limits.min_axle_torque.size()) {
                    std::ostringstream oss;
                    oss << "UserInput.axle_torques length " << axle_torques.size()
                        << " does not match driven axle count " << limits.min_axle_torque.size();
                    throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
                }

                for (std::size_t i = 0; i < axle_torques.size(); ++i) {
                    require_finite(axle_torques[i], "axle_torques");
                    require_in_range(axle_torques[i], "axle_torques", limits.min_axle_torque[i], limits.max_axle_torque[i]);
                }
            }
            break;
        }
        default:
            throw ::velox::errors::InputError(VELOX_LOC("Unknown UserInput control mode"));
    }

    if (drift_toggle.has_value()) {
        require_finite(*drift_toggle, "drift_toggle");
        require_in_range(*drift_toggle, "drift_toggle", limits.min_drift_toggle, limits.max_drift_toggle);
    }

}

namespace {
ModelInterface build_model_interface(ModelType model)
{
    ModelInterface iface{};
    switch (model) {
        case ModelType::MB:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters& params) {
                return models::init_mb(init_state, params);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                return models::vehicle_dynamics_mb(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                if (state.size() > 10) {
                    return std::hypot(state[3], state[10]);
                }
                if (state.size() > 3) {
                    return std::abs(state[3]);
                }
                return 0.0;
            };
            break;

        case ModelType::ST:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters&) {
                return models::init_st(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                return models::vehicle_dynamics_st(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::STD:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters& params) {
                return models::init_std(init_state, params);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double dt) {
                return models::vehicle_dynamics_std(x, u, params, dt);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        default:
            throw ::velox::errors::SimulationError(
                VELOX_MODEL("Unsupported model type for simulator", model));
    }
    return iface;
}

LowSpeedSafety build_low_speed_safety(ModelType model,
                                      const models::VehicleParameters& params,
                                      const LowSpeedSafetyConfig& cfg)
{
    std::optional<int> longitudinal;
    std::optional<int> lateral;
    std::optional<int> yaw_rate;
    std::optional<int> slip;
    std::vector<int>   wheel_indices;
    std::optional<int> steering;

    switch (model) {
        case ModelType::ST:
            longitudinal = 3;
            yaw_rate     = 5;
            slip         = 6;
            steering     = 2;
            break;

        case ModelType::STD:
            longitudinal = 3;
            yaw_rate     = 5;
            slip         = 6;
            wheel_indices = {7, 8};
            steering      = 2;
            break;

        case ModelType::MB:
            longitudinal = 3;
            lateral      = 10;
            yaw_rate     = 5;
            wheel_indices = {23, 24, 25, 26};
            steering      = 2;
            break;

        default:
            throw ::velox::errors::SimulationError(
                VELOX_MODEL("Unsupported model type for safety", model));
    }

    std::optional<double> wheelbase;
    std::optional<double> rear_length;
    if (std::isfinite(params.a) && std::isfinite(params.b)) {
        const double wb = params.a + params.b;
        if (wb > 0.0) {
            wheelbase = wb;
        }
    }
    if (std::isfinite(params.b) && params.b > 0.0) {
        rear_length = params.b;
    }

    return LowSpeedSafety(cfg,
                          longitudinal,
                          lateral,
                          yaw_rate,
                          slip,
                          wheel_indices,
                          steering,
                          wheelbase,
                          rear_length);
}

std::vector<double> seed_state(const ModelInterface& model_if, const models::VehicleParameters& params, const std::vector<double>& user)
{
    if (user.empty()) {
        return model_if.init_fn({}, params);
    }
    return model_if.init_fn(user, params);
}

} // namespace

SimulationDaemon::SimulationDaemon(const InitParams& init)
    : init_(init)
    , configs_(init.config_root, init.parameter_root)
    , model_(init.model)
    , timing_(configs_.load_model_timing(init.model))
    , control_mode_(init.control_mode)
{
    init_.use_default_log_sink();
    log_sink_ = init_.log_sink;

    load_vehicle_parameters(init.vehicle_id);

    const auto initial_safety_cfg = configs_.load_low_speed_safety_config(init.model);
    drift_enabled_                = init.drift_enabled.value_or(initial_safety_cfg.drift_enabled);
    init_.control_mode            = control_mode_;

    // Initial wiring occurs during construction
    const auto default_state = build_model_interface(model_).init_fn({}, params_);
    ResetParams reset_params{};
    reset_params.initial_state = default_state;
    reset_params.dt            = timing_.info().nominal_dt;
    reset(reset_params);
}

void SimulationDaemon::reset(const ResetParams& params)
{
    try {
        if (params.model) {
            model_ = *params.model;
        }
        if (params.vehicle_id) {
            init_.vehicle_id = *params.vehicle_id;
            load_vehicle_parameters(*params.vehicle_id);
        }
        if (params.control_mode) {
            control_mode_      = *params.control_mode;
            init_.control_mode = control_mode_;
        }

        timing_         = ModelTiming(configs_.load_model_timing(model_));
        model_interface_ = build_model_interface(model_);

        rebuild_controllers();

        const auto safety_cfg = configs_.load_low_speed_safety_config(model_);
        if (params.drift_enabled.has_value()) {
            drift_enabled_ = *params.drift_enabled;
        } else {
            drift_enabled_ = safety_cfg.drift_enabled;
        }
        init_.drift_enabled = drift_enabled_;
        init_.control_mode  = control_mode_;
        rebuild_safety(safety_cfg);

        const double initial_request = params.dt.value_or(timing_.info().nominal_dt);
        const auto   schedule        = timing_.plan_steps(initial_request);
        log_timing_adjustments(schedule);
        const auto   initial        = seed_state(model_interface_, params_, params.initial_state);

        rebuild_simulator(schedule.substeps.front(), initial);

        const double initial_angle = (initial.size() > 2) ? initial[2] : 0.0;
        if (steering_wheel_) {
            steering_wheel_->reset(initial_angle);
        }
        if (final_steer_) {
            final_steer_->reset(initial_angle);
        }
        if (accel_controller_) {
            accel_controller_->reset();
        }

        cumulative_distance_m_ = 0.0;
        cumulative_energy_j_   = 0.0;
        timing_.reset();
        last_telemetry_ = {};
    } catch (const ::velox::errors::VeloxError&) {
        throw;
    } catch (const std::exception& ex) {
        rethrow_with_context("reset", ex);
    }
}

telemetry::SimulationTelemetry SimulationDaemon::step(const UserInput& input)
{
    try {
        if (!simulator_ || !accel_controller_ || !steering_wheel_ || !final_steer_ || !safety_) {
            throw ::velox::errors::SimulationError(VELOX_MODEL("SimulationDaemon not initialized", model_));
        }

        UserInput working_input = input;
        working_input.control_mode = control_mode_;
        auto sanitized_input      = working_input.clamped(input_limits_);
        log_clamped_input(working_input, sanitized_input);

        if (sanitized_input.drift_toggle.has_value()) {
            set_drift_enabled(*sanitized_input.drift_toggle >= 0.5);
        }

        const auto schedule = timing_.plan_steps(sanitized_input.dt);
        log_timing_adjustments(schedule);

        controllers::longitudinal::ControllerOutput accel_output{};
        controllers::FinalSteerController::Output   final_output{};
        controllers::SteeringWheel::Output          steering_output{};

        switch (sanitized_input.control_mode) {
            case ControlMode::Keyboard: {
                for (const double dt : schedule.substeps) {
                    const double start_speed   = simulator_->speed();
                    const double current_angle = (simulator_->state().size() > 2) ? simulator_->state()[2] : 0.0;
                    steering_output = steering_wheel_->update(sanitized_input.steering_nudge, dt);
                    final_output = final_steer_->update(steering_output.target_angle, current_angle, dt);

                    const double speed = simulator_->speed();
                    accel_output = accel_controller_->step(sanitized_input.longitudinal, speed, dt);

                    std::vector<double> control{final_output.rate, accel_output.acceleration};
                    simulator_->set_dt(dt);
                    simulator_->step(control);

                    const double end_speed = simulator_->speed();
                    cumulative_distance_m_ += 0.5 * (std::abs(start_speed) + std::abs(end_speed)) * dt;
                    cumulative_energy_j_   += accel_output.battery_power * dt;
                    timing_.record_step(dt);
                }
                log_controller_limits(accel_output, final_output);
                break;
            }
            case ControlMode::Direct: {
                const double commanded_accel = direct_acceleration_from_torque(sanitized_input.axle_torques);
                const double total_force     = commanded_accel * params_.m;

                for (const double dt : schedule.substeps) {
                    const double start_speed   = simulator_->speed();
                    const double current_angle = (simulator_->state().size() > 2) ? simulator_->state()[2] : 0.0;

                    const double target_angle = sanitized_input.steering_angle;
                    final_output              = final_steer_->update_absolute(target_angle, current_angle, dt);
                    steering_output           = {target_angle, final_output.filtered_target, final_output.rate};

                    accel_output.acceleration    = commanded_accel;
                    accel_output.drive_force     = (total_force >= 0.0) ? total_force : 0.0;
                    accel_output.brake_force     = (total_force < 0.0) ? -total_force : 0.0;
                    accel_output.regen_force     = accel_output.brake_force;
                    accel_output.hydraulic_force = 0.0;

                    std::vector<double> control{final_output.rate, commanded_accel};
                    simulator_->set_dt(dt);
                    simulator_->step(control);

                    const double end_speed = simulator_->speed();
                    const double mean_speed = 0.5 * (std::abs(start_speed) + std::abs(end_speed));
                    accel_output.mechanical_power = total_force * mean_speed;
                    accel_output.battery_power    = accel_output.mechanical_power;

                    cumulative_distance_m_ += mean_speed * dt;
                    cumulative_energy_j_   += accel_output.battery_power * dt;
                    timing_.record_step(dt);
                }
                break;
            }
            default:
                throw ::velox::errors::InputError(VELOX_LOC("Unknown control mode"));
        }

        last_telemetry_ = compute_telemetry(accel_output, steering_output, final_output);
        return last_telemetry_;
    } catch (const ::velox::errors::VeloxError&) {
        throw;
    } catch (const std::exception& ex) {
        rethrow_with_context("step", ex);
    }
}

void SimulationDaemon::set_drift_enabled(bool enabled)
{
    drift_enabled_ = enabled;
    if (safety_) {
        safety_->set_drift_enabled(enabled);
    }
    if (simulator_) {
        simulator_->safety().set_drift_enabled(enabled);
    }
}

std::vector<telemetry::SimulationTelemetry> SimulationDaemon::step(const std::vector<UserInput>& batch_inputs)
{
    std::vector<telemetry::SimulationTelemetry> telemetry;
    telemetry.reserve(batch_inputs.size());
    for (std::size_t i = 0; i < batch_inputs.size(); ++i) {
        try {
            telemetry.push_back(step(batch_inputs[i]));
        } catch (const std::exception& ex) {
            std::ostringstream oss;
            oss << "batch input #" << i;
            rethrow_with_context("step", ex, oss.str());
        }
    }
    return telemetry;
}

SimulationSnapshot SimulationDaemon::snapshot() const
{
    SimulationSnapshot snap{};
    snap.telemetry = last_telemetry_;
    snap.simulation_time_s = timing_.cumulative_time();
    if (simulator_) {
        snap.state             = simulator_->state();
        snap.dt                = simulator_->dt();
    }
    return snap;
}

void SimulationDaemon::load_vehicle_parameters(int vehicle_id)
{
    params_ = configs_.load_vehicle_parameters(vehicle_id);
}

void SimulationDaemon::rebuild_controllers()
{
    const auto steering_cfg = configs_.load_steering_config();
    steering_wheel_.emplace(steering_cfg.wheel, params_.steering);
    final_steer_.emplace(steering_cfg.final, params_.steering);

    const auto aero_cfg         = configs_.load_aero_config();
    const auto rolling_cfg      = configs_.load_rolling_resistance_config();
    const auto brake_cfg        = configs_.load_brake_config();
    powertrain_config_          = configs_.load_powertrain_config();
    const auto accel_controller = configs_.load_final_accel_controller_config();

    accel_controller_.emplace(params_.m,
                              params_.R_w,
                              powertrain_config_,
                              aero_cfg,
                              rolling_cfg,
                              brake_cfg,
                              accel_controller);

    rebuild_drivetrain_layout();
    rebuild_input_limits();
}

void SimulationDaemon::rebuild_drivetrain_layout()
{
    drivetrain_layout_ = {};

    drivetrain_layout_.front_split = std::clamp(params_.T_se, 0.0, 1.0);
    drivetrain_layout_.rear_split  = std::clamp(1.0 - drivetrain_layout_.front_split, 0.0, 1.0);

    if (drivetrain_layout_.front_split > 0.0) {
        drivetrain_layout_.driven_axles.push_back(DrivenAxle::Front);
    }
    if (drivetrain_layout_.rear_split > 0.0) {
        drivetrain_layout_.driven_axles.push_back(DrivenAxle::Rear);
    }
}

void SimulationDaemon::rebuild_safety(const LowSpeedSafetyConfig& safety_cfg)
{
    safety_.emplace(build_low_speed_safety(model_, params_, safety_cfg));
    safety_->set_drift_enabled(drift_enabled_);
}

void SimulationDaemon::rebuild_simulator(double dt, const std::vector<double>& initial_state)
{
    if (!safety_) {
        throw ::velox::errors::SimulationError(VELOX_MODEL("Safety system not initialized", model_));
    }
    simulator_ = std::make_unique<VehicleSimulator>(model_interface_, params_, dt, *safety_);
    simulator_->reset(initial_state);
    simulator_->safety().set_drift_enabled(drift_enabled_);
}

void SimulationDaemon::rebuild_input_limits()
{
    input_limits_ = UserInputLimits::from_vehicle(
        steering_wheel_ ? &(*steering_wheel_) : nullptr,
        final_steer_ ? &(*final_steer_) : nullptr,
        params_,
        accel_controller_ ? &powertrain_config_ : nullptr);
}

double SimulationDaemon::direct_acceleration_from_torque(const std::vector<double>& axle_torques) const
{
    if (axle_torques.size() != drivetrain_layout_.driven_axles.size()) {
        std::ostringstream oss;
        oss << "Expected " << drivetrain_layout_.driven_axles.size() << " driven axle torques, got "
            << axle_torques.size();
        throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
    }

    if (drivetrain_layout_.driven_axles.empty()) {
        throw ::velox::errors::SimulationError(VELOX_LOC("No driven axles configured for direct torque control"));
    }

    if (!(params_.m > 0.0)) {
        throw ::velox::errors::SimulationError(VELOX_LOC("Vehicle mass must be positive for torque conversion"));
    }
    if (!(params_.R_w > 0.0)) {
        throw ::velox::errors::SimulationError(VELOX_LOC("Wheel radius must be positive for torque conversion"));
    }

    const double total_torque = std::accumulate(axle_torques.begin(), axle_torques.end(), 0.0);
    return total_torque / (params_.m * params_.R_w);
}

telemetry::SimulationTelemetry SimulationDaemon::compute_telemetry(
    const controllers::longitudinal::ControllerOutput& accel_output,
    const controllers::SteeringWheel::Output& steering_input,
    const controllers::FinalSteerController::Output& steering_output) const
{
    const auto* simulator_ptr = simulator_.get();
    const auto* safety_ptr    = simulator_ptr ? &simulator_ptr->safety() : safety_ ? &(*safety_) : nullptr;
    const auto& state         = simulator_ptr ? simulator_ptr->state() : std::vector<double>{};
    const double measured_speed = simulator_ptr ? simulator_ptr->speed() : 0.0;

    return telemetry::compute_simulation_telemetry(model_,
                                                   params_,
                                                   state,
                                                   accel_output,
                                                   steering_input,
                                                   steering_output,
                                                   safety_ptr,
                                                   measured_speed,
                                                   cumulative_distance_m_,
                                                   cumulative_energy_j_,
                                                   timing_.cumulative_time());
}

void SimulationDaemon::log_warning(const std::string& message) const
{
    logging::log(log_sink_.get(), logging::Level::Warning, message);
}

void SimulationDaemon::log_info(const std::string& message) const
{
    logging::log(log_sink_.get(), logging::Level::Info, message);
}

void SimulationDaemon::log_clamped_input(const UserInput& original, const UserInput& clamped) const
{
    const auto changed = [](double a, double b) { return std::abs(a - b) > 1e-9; };
    if (changed(original.longitudinal.throttle, clamped.longitudinal.throttle)) {
        std::ostringstream oss;
        oss << "Throttle request clamped from " << original.longitudinal.throttle << " to "
            << clamped.longitudinal.throttle << " within [" << input_limits_.min_throttle << ", "
            << input_limits_.max_throttle << "]";
        log_warning(oss.str());
    }
    if (changed(original.longitudinal.brake, clamped.longitudinal.brake)) {
        std::ostringstream oss;
        oss << "Brake request clamped from " << original.longitudinal.brake << " to "
            << clamped.longitudinal.brake << " within [" << input_limits_.min_brake << ", "
            << input_limits_.max_brake << "]";
        log_warning(oss.str());
    }
    if (changed(original.steering_nudge, clamped.steering_nudge)) {
        std::ostringstream oss;
        oss << "Steering nudge clamped from " << original.steering_nudge << " to "
            << clamped.steering_nudge << " within [" << input_limits_.min_steering_nudge << ", "
            << input_limits_.max_steering_nudge << "]";
        log_warning(oss.str());
    }
    if (changed(original.steering_angle, clamped.steering_angle)) {
        std::ostringstream oss;
        oss << "Steering angle clamped from " << original.steering_angle << " to " << clamped.steering_angle
            << " within [" << input_limits_.min_steering_angle << ", " << input_limits_.max_steering_angle << "]";
        log_warning(oss.str());
    }

    const auto torque_bounds_match = input_limits_.min_axle_torque.size() == input_limits_.max_axle_torque.size();
    if (torque_bounds_match && clamped.axle_torques.size() == input_limits_.min_axle_torque.size()) {
        for (std::size_t i = 0; i < clamped.axle_torques.size(); ++i) {
            if (changed(original.axle_torques[i], clamped.axle_torques[i])) {
                std::ostringstream oss;
                oss << "Axle torque #" << i << " clamped from " << original.axle_torques[i] << " to "
                    << clamped.axle_torques[i] << " within [" << input_limits_.min_axle_torque[i] << ", "
                    << input_limits_.max_axle_torque[i] << "]";
                log_warning(oss.str());
            }
        }
    }
}

void SimulationDaemon::log_controller_limits(const controllers::longitudinal::ControllerOutput& accel_output,
                                             const controllers::FinalSteerController::Output& steer_output) const
{
    constexpr double kTol = 1e-6;
    if (accel_controller_) {
        const auto& cfg = accel_controller_->config();
        if (accel_output.acceleration <= cfg.accel_min + kTol) {
            std::ostringstream oss;
            oss << "Acceleration saturated at lower bound " << cfg.accel_min << " m/s^2";
            log_warning(oss.str());
        } else if (accel_output.acceleration >= cfg.accel_max - kTol) {
            std::ostringstream oss;
            oss << "Acceleration saturated at upper bound " << cfg.accel_max << " m/s^2";
            log_warning(oss.str());
        }
    }

    if (final_steer_) {
        const auto& cfg = final_steer_->config();
        if (std::abs(steer_output.rate - cfg.max_rate) < kTol ||
            std::abs(steer_output.rate + cfg.max_rate) < kTol) {
            std::ostringstream oss;
            oss << "Steering rate saturated at ±" << cfg.max_rate << " rad/s";
            log_warning(oss.str());
        }
        if (steer_output.angle <= cfg.min_angle + kTol || steer_output.angle >= cfg.max_angle - kTol) {
            std::ostringstream oss;
            oss << "Steering angle limited within [" << cfg.min_angle << ", " << cfg.max_angle << "]";
            log_warning(oss.str());
        }
    }
}

void SimulationDaemon::log_timing_adjustments(const ModelTiming::StepSchedule& schedule) const
{
    if (schedule.clamped_to_min) {
        std::ostringstream oss;
        oss << "Requested dt " << schedule.requested_dt << "s raised to minimum stable timestep "
            << schedule.clamped_dt << "s";
        log_warning(oss.str());
    }

    if (schedule.used_substeps && !schedule.substeps.empty()) {
        const auto max_dt = *std::max_element(schedule.substeps.begin(), schedule.substeps.end());
        std::ostringstream oss;
        oss << "Requested dt " << schedule.requested_dt << "s exceeds stable max " << timing_.info().max_dt
            << "s; using " << schedule.substeps.size() << " sub-steps (max " << max_dt << "s) for stability";
        log_warning(oss.str());
    }
}

std::string SimulationDaemon::context_description(std::string_view action) const
{
    std::ostringstream oss;
    oss << "SimulationDaemon " << action << " failed";
    oss << " for model " << model_display_name(model_);
    oss << " (vehicle_id=" << init_.vehicle_id << ")";
    return oss.str();
}

[[noreturn]] void SimulationDaemon::rethrow_with_context(const char* action,
                                                         const std::exception& ex,
                                                         std::string_view index_context) const
{
    auto context = context_description(action);
    if (!index_context.empty()) {
        context = ::velox::errors::append_context(std::move(context), index_context);
    }

    throw ::velox::errors::SimulationError(
        VELOX_MODEL(::velox::errors::append_context(ex.what(), context), model_));
}

[[noreturn]] void SimulationDaemon::rethrow_with_context(const char* action, const std::exception& ex) const
{
    rethrow_with_context(action, ex, {});
}

} // namespace velox::simulation

