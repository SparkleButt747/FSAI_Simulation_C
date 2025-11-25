#include "telemetry/telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace velox::telemetry {

namespace {
constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-6;

double yaw_rate_from_state(simulation::ModelType model, const std::vector<double>& state, double v_long, double steer_angle, double wheelbase)
{
    (void)v_long;
    (void)steer_angle;
    (void)wheelbase;

    switch (model) {
        case simulation::ModelType::ST:
        case simulation::ModelType::STD:
        case simulation::ModelType::MB:
            if (state.size() > 5) {
                return state[5];
            }
            break;
    }
    return 0.0;
}

double axle_slip_angle(double v_lat, double v_long, double yaw_rate, double offset, double steer_angle = 0.0)
{
    const double safe_longitudinal = (std::abs(v_long) > kEpsilon) ? v_long : std::copysign(kEpsilon, v_long + kEpsilon);
    const double lateral_velocity  = v_lat + yaw_rate * offset;
    return std::atan2(lateral_velocity, safe_longitudinal) - steer_angle;
}

const char* safety_stage_name(simulation::SafetyStage stage)
{
    switch (stage) {
        case simulation::SafetyStage::Normal:
            return "normal";
        case simulation::SafetyStage::Transition:
            return "transition";
        case simulation::SafetyStage::Emergency:
            return "emergency";
    }
    return "unknown";
}

std::pair<WheelTelemetry, WheelTelemetry> build_wheels(double base_longitudinal_velocity,
                                                       double v_lat,
                                                       double yaw_rate,
                                                       double track_width,
                                                       double steer_angle,
                                                       double axle_force,
                                                       double normal_force)
{
    const double half_track  = 0.5 * std::max(track_width, 0.0);
    const double left_offset = half_track;
    const double right_offset = -half_track;

    WheelTelemetry left{};
    WheelTelemetry right{};

    const double left_speed_body  = base_longitudinal_velocity - yaw_rate * left_offset;
    const double right_speed_body = base_longitudinal_velocity - yaw_rate * right_offset;

    const double oriented_left  = left_speed_body * std::cos(steer_angle) + v_lat * std::sin(steer_angle);
    const double oriented_right = right_speed_body * std::cos(steer_angle) + v_lat * std::sin(steer_angle);

    const double reference_speed = std::max(std::abs(base_longitudinal_velocity), kEpsilon);

    left.speed      = oriented_left;
    right.speed     = oriented_right;
    left.slip_ratio = (oriented_left - base_longitudinal_velocity) / reference_speed;
    right.slip_ratio = (oriented_right - base_longitudinal_velocity) / reference_speed;

    const double per_wheel_force = 0.5 * axle_force;
    const double per_wheel_normal = 0.5 * std::max(normal_force, kEpsilon);
    left.friction_utilization  = std::abs(per_wheel_force) / per_wheel_normal;
    right.friction_utilization = std::abs(per_wheel_force) / per_wheel_normal;

    return {left, right};
}

void populate_axle(AxleTelemetry& axle,
                   double drive_force,
                   double brake_force,
                   double regen_force,
                   double drive_split,
                   double brake_split,
                   double wheel_radius,
                   double normal_force)
{
    const double drive = drive_force * std::clamp(drive_split, 0.0, 1.0);
    const double brake = brake_force * std::clamp(brake_split, 0.0, 1.0);
    const double regen = regen_force * std::clamp(brake_split, 0.0, 1.0);

    axle.drive_torque = drive * wheel_radius;
    axle.brake_torque = brake * wheel_radius;
    axle.regen_torque = regen * wheel_radius;
    axle.normal_force = normal_force;
}

} // namespace

SimulationTelemetry compute_simulation_telemetry(
    simulation::ModelType model,
    const models::VehicleParameters& params,
    const std::vector<double>& state,
    const controllers::longitudinal::ControllerOutput& accel_output,
    const controllers::SteeringWheel::Output& steering_wheel_output,
    const controllers::FinalSteerController::Output& steering_output,
    const simulation::LowSpeedSafety* safety,
    double measured_speed,
    double cumulative_distance_m,
    double cumulative_energy_j,
    double cumulative_sim_time_s)
{
    SimulationTelemetry telemetry{};

    const double wheelbase = params.a + params.b;
    const double steer_angle = (state.size() > 2) ? state[2] : steering_output.angle;

    double v_long = 0.0;
    double v_lat  = 0.0;
    double yaw    = 0.0;
    double beta   = 0.0;

    switch (model) {
        case simulation::ModelType::ST:
            if (state.size() >= 7) {
                v_long = state[3];
                yaw    = state[4];
                beta   = state[6];
            }
            break;

        case simulation::ModelType::STD:
            if (state.size() >= 9) {
                v_long = state[3];
                yaw    = state[4];
                beta   = state[6];
            }
            break;

        case simulation::ModelType::MB:
            if (state.size() >= 11) {
                v_long = state[3];
                v_lat  = state[10];
                yaw    = state[4];
            }
            if (std::abs(v_long) > kEpsilon || std::abs(v_lat) > kEpsilon) {
                beta = std::atan2(v_lat, v_long);
            }
            break;
    }

    telemetry.pose.x   = (state.size() > 0) ? state[0] : 0.0;
    telemetry.pose.y   = (state.size() > 1) ? state[1] : 0.0;
    telemetry.pose.yaw = yaw;

    const double speed   = (measured_speed > 0.0) ? measured_speed : std::hypot(v_long, v_lat);
    const double heading = yaw + beta;
    const double yaw_rate = yaw_rate_from_state(model, state, v_long, steer_angle, wheelbase);

    telemetry.velocity.speed        = speed;
    telemetry.velocity.longitudinal = v_long;
    telemetry.velocity.lateral      = v_lat;
    telemetry.velocity.yaw_rate     = yaw_rate;
    telemetry.velocity.global_x     = speed * std::cos(heading);
    telemetry.velocity.global_y     = speed * std::sin(heading);

    telemetry.traction.slip_angle = beta;

    telemetry.acceleration.longitudinal = accel_output.acceleration;
    switch (model) {
        case simulation::ModelType::MB:
            if (state.size() > 5) {
                telemetry.acceleration.lateral = v_long * state[5];
            }
            break;
        case simulation::ModelType::ST:
        case simulation::ModelType::STD:
            if (state.size() > 5) {
                telemetry.acceleration.lateral = v_long * state[5];
            }
            break;
    }

    telemetry.traction.front_slip_angle = axle_slip_angle(v_lat, v_long, yaw_rate, params.a, steer_angle);
    telemetry.traction.rear_slip_angle  = axle_slip_angle(v_lat, v_long, yaw_rate, -params.b);

    telemetry.steering.desired_angle = steering_wheel_output.target_angle;
    telemetry.steering.desired_rate  = steering_wheel_output.rate;
    telemetry.steering.actual_angle  = steering_output.angle;
    telemetry.steering.actual_rate   = steering_output.rate;

    telemetry.controller.acceleration    = accel_output.acceleration;
    telemetry.controller.throttle        = accel_output.throttle;
    telemetry.controller.brake           = accel_output.brake;
    telemetry.controller.drive_force     = accel_output.drive_force;
    telemetry.controller.brake_force     = accel_output.brake_force;
    telemetry.controller.regen_force     = accel_output.regen_force;
    telemetry.controller.hydraulic_force = accel_output.hydraulic_force;
    telemetry.controller.drag_force      = accel_output.drag_force;
    telemetry.controller.rolling_force   = accel_output.rolling_force;

    telemetry.powertrain.total_torque     = (accel_output.drive_force - accel_output.regen_force) * params.R_w;
    telemetry.powertrain.drive_torque     = accel_output.drive_force * params.R_w;
    telemetry.powertrain.regen_torque     = accel_output.regen_force * params.R_w;
    telemetry.powertrain.mechanical_power = accel_output.mechanical_power;
    telemetry.powertrain.battery_power    = accel_output.battery_power;
    telemetry.powertrain.soc              = accel_output.soc;

    const double normal_front = (wheelbase > kEpsilon)
        ? params.m * kGravity * (params.b / wheelbase)
        : 0.5 * params.m * kGravity;
    const double normal_rear = (wheelbase > kEpsilon)
        ? params.m * kGravity * (params.a / wheelbase)
        : 0.5 * params.m * kGravity;

    populate_axle(telemetry.front_axle,
                  accel_output.drive_force,
                  accel_output.brake_force,
                  accel_output.regen_force,
                  params.T_se,
                  params.T_sb,
                  params.R_w,
                  normal_front);
    populate_axle(telemetry.rear_axle,
                  accel_output.drive_force,
                  accel_output.brake_force,
                  accel_output.regen_force,
                  1.0 - params.T_se,
                  1.0 - params.T_sb,
                  params.R_w,
                  normal_rear);

    const double front_net_force = accel_output.drive_force * params.T_se
        - accel_output.brake_force * params.T_sb;
    const double rear_net_force  = accel_output.drive_force * (1.0 - params.T_se)
        - accel_output.brake_force * (1.0 - params.T_sb);

    const auto [front_left, front_right] = build_wheels(v_long,
                                                        v_lat,
                                                        yaw_rate,
                                                        params.T_f,
                                                        steer_angle,
                                                        front_net_force,
                                                        normal_front);
    telemetry.front_axle.left  = front_left;
    telemetry.front_axle.right = front_right;

    const auto [rear_left, rear_right] = build_wheels(v_long,
                                                      v_lat,
                                                      yaw_rate,
                                                      params.T_r,
                                                      0.0,
                                                      rear_net_force,
                                                      normal_rear);
    telemetry.rear_axle.left  = rear_left;
    telemetry.rear_axle.right = rear_right;

    const double total_normal      = normal_front + normal_rear;
    const double lateral_available = std::max(params.tire.p_dy1 * total_normal, kEpsilon);
    const double lateral_force     = params.m * telemetry.acceleration.lateral;
    telemetry.traction.lateral_force_saturation = std::abs(lateral_force) / lateral_available;

    telemetry.totals.distance_traveled_m    = cumulative_distance_m;
    telemetry.totals.energy_consumed_joules = cumulative_energy_j;
    telemetry.totals.simulation_time_s      = cumulative_sim_time_s;
    if (safety) {
        const auto status = safety->status(state, speed);
        telemetry.detector_severity = status.severity;
        telemetry.safety_stage      = status.stage;
        telemetry.detector_forced   = status.detector_forced;
        telemetry.low_speed_engaged = status.latch_active;
        telemetry.traction.drift_mode = status.drift_mode;
    } else {
        telemetry.low_speed_engaged = false;
        telemetry.traction.drift_mode = false;
    }

    return telemetry;
}

const char* safety_stage_to_string(simulation::SafetyStage stage)
{
    return safety_stage_name(stage);
}

std::string to_json(const SimulationTelemetry& telemetry)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    oss << '{'
        << "\"pose\":{"
        << "\"x\":" << telemetry.pose.x << ","
        << "\"y\":" << telemetry.pose.y << ","
        << "\"yaw\":" << telemetry.pose.yaw << "},";

    oss << "\"velocity\":{"
        << "\"speed\":" << telemetry.velocity.speed << ","
        << "\"longitudinal\":" << telemetry.velocity.longitudinal << ","
        << "\"lateral\":" << telemetry.velocity.lateral << ","
        << "\"yaw_rate\":" << telemetry.velocity.yaw_rate << ","
        << "\"global_x\":" << telemetry.velocity.global_x << ","
        << "\"global_y\":" << telemetry.velocity.global_y << "},";

    oss << "\"acceleration\":{"
        << "\"longitudinal\":" << telemetry.acceleration.longitudinal << ","
        << "\"lateral\":" << telemetry.acceleration.lateral << "},";

    oss << "\"traction\":{"
        << "\"slip_angle\":" << telemetry.traction.slip_angle << ","
        << "\"front_slip_angle\":" << telemetry.traction.front_slip_angle << ","
        << "\"rear_slip_angle\":" << telemetry.traction.rear_slip_angle << ","
        << "\"lateral_force_saturation\":" << telemetry.traction.lateral_force_saturation << ","
        << "\"drift_mode\":" << (telemetry.traction.drift_mode ? "true" : "false") << "},";

    auto write_wheel = [&oss](const char* name, const WheelTelemetry& wheel) {
        oss << '\"' << name << "\":"
            << "{\"speed\":" << wheel.speed << ","
            << "\"slip_ratio\":" << wheel.slip_ratio << ","
            << "\"friction_utilization\":" << wheel.friction_utilization << "}";
    };

    auto write_axle = [&oss, &write_wheel](const char* name, const AxleTelemetry& axle) {
        oss << '\"' << name << "\":{"
            << "\"drive_torque\":" << axle.drive_torque << ","
            << "\"brake_torque\":" << axle.brake_torque << ","
            << "\"regen_torque\":" << axle.regen_torque << ","
            << "\"normal_force\":" << axle.normal_force << ',';
        write_wheel("left", axle.left);
        oss << ',';
        write_wheel("right", axle.right);
        oss << '}';
    };

    oss << "\"steering\":{"
        << "\"desired_angle\":" << telemetry.steering.desired_angle << ","
        << "\"desired_rate\":" << telemetry.steering.desired_rate << ","
        << "\"actual_angle\":" << telemetry.steering.actual_angle << ","
        << "\"actual_rate\":" << telemetry.steering.actual_rate << "},";

    oss << "\"controller\":{"
        << "\"acceleration\":" << telemetry.controller.acceleration << ","
        << "\"throttle\":" << telemetry.controller.throttle << ","
        << "\"brake\":" << telemetry.controller.brake << ","
        << "\"drive_force\":" << telemetry.controller.drive_force << ","
        << "\"brake_force\":" << telemetry.controller.brake_force << ","
        << "\"regen_force\":" << telemetry.controller.regen_force << ","
        << "\"hydraulic_force\":" << telemetry.controller.hydraulic_force << ","
        << "\"drag_force\":" << telemetry.controller.drag_force << ","
        << "\"rolling_force\":" << telemetry.controller.rolling_force << "},";

    oss << "\"powertrain\":{"
        << "\"total_torque\":" << telemetry.powertrain.total_torque << ","
        << "\"drive_torque\":" << telemetry.powertrain.drive_torque << ","
        << "\"regen_torque\":" << telemetry.powertrain.regen_torque << ","
        << "\"mechanical_power\":" << telemetry.powertrain.mechanical_power << ","
        << "\"battery_power\":" << telemetry.powertrain.battery_power << ","
        << "\"soc\":" << telemetry.powertrain.soc << "},";

    write_axle("front_axle", telemetry.front_axle);
    oss << ',';
    write_axle("rear_axle", telemetry.rear_axle);
    oss << ',';

    oss << "\"totals\":{"
        << "\"distance_traveled_m\":" << telemetry.totals.distance_traveled_m << ","
        << "\"energy_consumed_joules\":" << telemetry.totals.energy_consumed_joules << ","
        << "\"simulation_time_s\":" << telemetry.totals.simulation_time_s << "},";

    oss << "\"low_speed_engaged\":" << (telemetry.low_speed_engaged ? "true" : "false") << ','
        << "\"detector_severity\":" << telemetry.detector_severity << ','
        << "\"safety_stage\":\"" << safety_stage_name(telemetry.safety_stage) << "\","
        << "\"detector_forced\":" << (telemetry.detector_forced ? "true" : "false")
        << '}';

    return oss.str();
}


} // namespace velox::telemetry

