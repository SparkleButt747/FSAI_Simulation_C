#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "common/logging.hpp"
#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/steering_controller.hpp"
#include "io/config_manager.hpp"
#include "simulation/low_speed_safety.hpp"
#include "simulation/model_timing.hpp"
#include "simulation/vehicle_simulator.hpp"
#include "telemetry/telemetry.hpp"
#include "simulation/user_input.hpp"

namespace velox::simulation {

struct ResetParams {
    std::optional<ModelType> model{};
    std::optional<int>       vehicle_id{};
    std::vector<double>      initial_state{};
    std::optional<double>    dt{};
    std::optional<bool>      drift_enabled{};
    std::optional<ControlMode> control_mode{};
};

struct SimulationSnapshot {
    std::vector<double>               state{};
    telemetry::SimulationTelemetry    telemetry{};
    double                            dt{0.0};
    double                            simulation_time_s{0.0};
};

    class SimulationDaemon {
    public:
        struct InitParams {
            ModelType                 model{ModelType::MB};
            int                       vehicle_id{1};
            std::filesystem::path     config_root{};
            std::filesystem::path     parameter_root{};
            logging::LogSinkPtr       log_sink{};
            std::optional<bool>       drift_enabled{};
            ControlMode               control_mode{ControlMode::Keyboard};

            void use_default_log_sink()
            {
                if (!log_sink) {
                    log_sink = logging::make_console_log_sink();
                }
            }
        };

    explicit SimulationDaemon(const InitParams& init);

    void reset(const ResetParams& params);

    void set_log_sink(logging::LogSinkPtr sink) { log_sink_ = std::move(sink); }

    telemetry::SimulationTelemetry step(const UserInput& input);
    std::vector<telemetry::SimulationTelemetry> step(const std::vector<UserInput>& batch_inputs);

    [[nodiscard]] const VehicleSimulator* simulator() const { return simulator_.get(); }
    [[nodiscard]] const controllers::longitudinal::FinalAccelController* accel_controller() const
    {
        return accel_controller_ ? &(*accel_controller_) : nullptr;
    }
    [[nodiscard]] const controllers::FinalSteerController* steering_controller() const
    {
        return final_steer_ ? &(*final_steer_) : nullptr;
    }
    [[nodiscard]] const controllers::SteeringWheel* steering_wheel() const
    {
        return steering_wheel_ ? &(*steering_wheel_) : nullptr;
    }
    [[nodiscard]] const models::VehicleParameters& vehicle_parameters() const { return params_; }
    [[nodiscard]] ModelType model() const { return model_; }
    [[nodiscard]] bool drift_enabled() const { return drift_enabled_; }
    [[nodiscard]] ControlMode control_mode() const { return control_mode_; }
    void                set_drift_enabled(bool enabled);
    [[nodiscard]] const telemetry::SimulationTelemetry& telemetry() const { return last_telemetry_; }

    [[nodiscard]] SimulationSnapshot snapshot() const;

private:
    enum class DrivenAxle
    {
        Front,
        Rear,
    };

    struct DrivetrainLayout
    {
        std::vector<DrivenAxle> driven_axles{};
        double                  front_split{0.0};
        double                  rear_split{0.0};
    };

    InitParams              init_{};
    io::ConfigManager       configs_{};
    ModelType               model_{};
    ModelTiming             timing_{};
    bool                    drift_enabled_{false};

    models::VehicleParameters                              params_{};
    ModelInterface                                          model_interface_{};
    std::optional<LowSpeedSafety>                          safety_{};
    std::unique_ptr<VehicleSimulator>                      simulator_{};

    std::optional<controllers::SteeringWheel>             steering_wheel_{};
    std::optional<controllers::FinalSteerController>       final_steer_{};
    std::optional<controllers::longitudinal::FinalAccelController> accel_controller_{};
    controllers::longitudinal::PowertrainConfig            powertrain_config_{};
    UserInputLimits                                        input_limits_{kDefaultUserInputLimits};
    DrivetrainLayout                                        drivetrain_layout_{};

    ControlMode control_mode_{ControlMode::Keyboard};

    logging::LogSinkPtr log_sink_{};

    double cumulative_distance_m_{0.0};
    double cumulative_energy_j_{0.0};

    telemetry::SimulationTelemetry last_telemetry_{};

    void load_vehicle_parameters(int vehicle_id);
    void rebuild_controllers();
    void rebuild_drivetrain_layout();
    void rebuild_safety(const LowSpeedSafetyConfig& safety_cfg);
    void rebuild_simulator(double dt, const std::vector<double>& initial_state);
    void rebuild_input_limits();
    double direct_acceleration_from_torque(const std::vector<double>& axle_torques) const;
    telemetry::SimulationTelemetry compute_telemetry(
        const controllers::longitudinal::ControllerOutput& accel_output,
        const controllers::SteeringWheel::Output& steering_input,
        const controllers::FinalSteerController::Output&    steering_output) const;
    void log_warning(const std::string& message) const;
    void log_info(const std::string& message) const;
    void log_clamped_input(const UserInput& original, const UserInput& clamped) const;
    void log_controller_limits(const controllers::longitudinal::ControllerOutput& accel_output,
                               const controllers::FinalSteerController::Output& steer_output) const;
    void log_timing_adjustments(const ModelTiming::StepSchedule& schedule) const;
    std::string context_description(std::string_view action) const;
    [[noreturn]] void rethrow_with_context(const char* action,
                                           const std::exception& ex,
                                           std::string_view index_context) const;
    [[noreturn]] void rethrow_with_context(const char* action, const std::exception& ex) const;
};

} // namespace velox::simulation

