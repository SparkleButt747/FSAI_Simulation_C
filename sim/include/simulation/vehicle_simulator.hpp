#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "simulation/low_speed_safety.hpp"
#include "vehicle_parameters.hpp"

namespace velox::simulation {

using ::velox::models::VehicleParameters;

struct ModelInterface {
    using State = std::vector<double>;
    using Control = std::vector<double>;
    using InitFunction = std::function<State(const State&, const VehicleParameters&)>;
    using DynamicsFunction =
        std::function<State(const State&, const Control&, const VehicleParameters&, double)>;
    using SpeedFunction = std::function<double(const State&, const VehicleParameters&)>;

    InitFunction init_fn;
    DynamicsFunction dynamics_fn;
    SpeedFunction    speed_fn;

    bool valid() const
    {
        return static_cast<bool>(init_fn) && static_cast<bool>(dynamics_fn) && static_cast<bool>(speed_fn);
    }
};

class VehicleSimulator {
public:
    VehicleSimulator(ModelInterface model,
                     VehicleParameters params,
                     double dt,
                     LowSpeedSafety safety);

    void reset(const std::vector<double>& initial_state);

    const std::vector<double>& state() const;
    double speed() const;

    const std::vector<double>& step(const std::vector<double>& control);

    void set_dt(double dt);
    double dt() const { return dt_; }

    void seed_state(const std::vector<double>& state);

    LowSpeedSafety& safety() { return safety_; }
    const LowSpeedSafety& safety() const { return safety_; }

private:
    ModelInterface model_;
    VehicleParameters params_;
    double dt_ = 0.0;
    LowSpeedSafety safety_;
    std::vector<double> state_;
    bool ready_ = false;

    void ensure_ready() const;
    std::vector<double> add_scaled(const std::vector<double>& base,
                                   double scale,
                                   const std::vector<double>& delta) const;
    std::pair<std::vector<double>, std::vector<double>> dynamics(const std::vector<double>& state,
                                                                 const std::vector<double>& control,
                                                                 bool update_latch);
    void apply_safety(std::vector<double>& state, bool update_latch);
};

} // namespace velox::simulation
