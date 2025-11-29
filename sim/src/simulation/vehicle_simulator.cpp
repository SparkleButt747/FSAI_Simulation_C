#include "simulation/vehicle_simulator.hpp"

#include <stdexcept>

#include "common/errors.hpp"
namespace velox::simulation {

VehicleSimulator::VehicleSimulator(ModelInterface model,
                                   VehicleParameters params,
                                   double dt,
                                   LowSpeedSafety safety)
    : model_(std::move(model))
    , params_(std::move(params))
    , safety_(std::move(safety))
{
    if (!model_.valid()) {
        throw ::velox::errors::SimulationError(VELOX_LOC("VehicleSimulator requires a valid ModelInterface"));
    }
    set_dt(dt);
    safety_.reset();
}

void VehicleSimulator::reset(const std::vector<double>& initial_state)
{
    safety_.reset();
    state_ = model_.init_fn(initial_state, params_);
    apply_safety(state_, true);
    ready_ = true;
}

double VehicleSimulator::speed() const
{
    ensure_ready();
    return model_.speed_fn(state_, params_);
}

const std::vector<double>& VehicleSimulator::state() const
{
    ensure_ready();
    return state_;
}

const std::vector<double>& VehicleSimulator::step(const std::vector<double>& control)
{
    ensure_ready();
    if (control.size() != 2) {
        throw ::velox::errors::InputError(VELOX_LOC("VehicleSimulator control must contain steering rate and acceleration"));
    }

    const double dt = dt_;
    const auto previous_state = state_;

    auto [k1, current] = dynamics(previous_state, control, true);
    state_ = current;

    auto k2_state = add_scaled(state_, 0.5 * dt, k1);
    auto [k2, _k2_state] = dynamics(k2_state, control, false);
    (void)_k2_state;

    auto k3_state = add_scaled(state_, 0.5 * dt, k2);
    auto [k3, _k3_state] = dynamics(k3_state, control, false);
    (void)_k3_state;

    auto k4_state = add_scaled(state_, dt, k3);
    auto [k4, _k4_state] = dynamics(k4_state, control, false);
    (void)_k4_state;

    if (k1.size() != state_.size() || k2.size() != state_.size() ||
        k3.size() != state_.size() || k4.size() != state_.size()) {
        throw ::velox::errors::SimulationError(VELOX_LOC("VehicleSimulator dynamics returned mismatched state dimension"));
    }

    for (std::size_t i = 0; i < state_.size(); ++i) {
        state_[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    if (const auto long_index = safety_.longitudinal_index()) {
        const std::size_t idx = static_cast<std::size_t>(*long_index);
        if (idx < previous_state.size() && idx < state_.size()) {
            const double prev_long = previous_state[idx];
            const double curr_long = state_[idx];
            if (prev_long >= 0.0 && curr_long < 0.0) {
                state_[idx] = 0.0;
            }
        }
    }

    apply_safety(state_, true);
    return state_;
}

void VehicleSimulator::set_dt(double dt)
{
    if (!(dt > 0.0)) {
        throw ::velox::errors::InputError(VELOX_LOC("VehicleSimulator timestep must be positive"));
    }
    dt_ = dt;
}

void VehicleSimulator::seed_state(const std::vector<double>& state)
{
    state_ = state;
    apply_safety(state_, true);
    ready_ = true;
}

void VehicleSimulator::ensure_ready() const
{
    if (!ready_) {
        throw ::velox::errors::SimulationError(VELOX_LOC("VehicleSimulator has not been initialised; call reset() first"));
    }
}

std::vector<double> VehicleSimulator::add_scaled(const std::vector<double>& base,
                                                 double scale,
                                                 const std::vector<double>& delta) const
{
    if (base.size() != delta.size()) {
        throw ::velox::errors::SimulationError(VELOX_LOC("VehicleSimulator::add_scaled size mismatch"));
    }
    std::vector<double> result(base.size());
    for (std::size_t i = 0; i < base.size(); ++i) {
        result[i] = base[i] + scale * delta[i];
    }
    return result;
}

std::pair<std::vector<double>, std::vector<double>> VehicleSimulator::dynamics(const std::vector<double>& state,
                                                                               const std::vector<double>& control,
                                                                               bool update_latch)
{
    std::vector<double> sanitized = state;
    apply_safety(sanitized, update_latch);
    auto rhs = model_.dynamics_fn(sanitized, control, params_, dt_);
    return {std::move(rhs), std::move(sanitized)};
}

void VehicleSimulator::apply_safety(std::vector<double>& state, bool update_latch)
{
    const double speed = model_.speed_fn(state, params_);
    safety_.apply(state, speed, update_latch);
}

} // namespace velox::simulation
