#include "simulation/model_timing.hpp"

#include <cmath>
#include <numeric>
#include <sstream>

#include "common/errors.hpp"

namespace velox::simulation {

ModelTiming::ModelTiming(ModelTimingInfo info, double start_time_s)
    : info_(info)
    , cumulative_time_s_(start_time_s)
{}

void ModelTiming::reset(double start_time_s)
{
    cumulative_time_s_ = start_time_s;
}

ModelTiming::StepSchedule ModelTiming::plan_steps(double requested_dt) const
{
    StepSchedule schedule{};
    schedule.requested_dt = requested_dt;

    if (!std::isfinite(requested_dt)) {
        throw ::velox::errors::InputError(VELOX_LOC("Requested dt must be finite"));
    }
    if (requested_dt <= 0.0) {
        std::ostringstream oss;
        oss << "Requested dt must be positive; got " << requested_dt;
        throw ::velox::errors::InputError(VELOX_LOC(oss.str()));
    }
    if (!(info_.max_dt > 0.0)) {
        throw ::velox::errors::SimulationError(
            VELOX_LOC("ModelTiming requires a positive max_dt configuration"));
    }

    const double total_dt = std::max(requested_dt, static_cast<double>(kMinStableDt));
    schedule.clamped_dt   = total_dt;
    schedule.clamped_to_min = total_dt > requested_dt;

    int steps = static_cast<int>(std::ceil(total_dt / static_cast<double>(info_.max_dt)));
    steps     = std::max(1, steps);
    while (steps > 1 && total_dt / static_cast<double>(steps) < static_cast<double>(kMinStableDt)) {
        --steps;
    }

    const double base_dt = total_dt / static_cast<double>(steps);
    double       accumulated{0.0};
    for (int i = 0; i < steps; ++i) {
        const bool   last   = (i == steps - 1);
        const double dt     = last ? (total_dt - accumulated) : base_dt;
        accumulated += dt;
        schedule.substeps.push_back(dt);
    }

    schedule.used_substeps = schedule.substeps.size() > 1;
    return schedule;
}

void ModelTiming::record_step(double dt)
{
    cumulative_time_s_ += dt;
}

double ModelTiming::StepSchedule::total_duration() const
{
    return std::accumulate(substeps.begin(), substeps.end(), 0.0);
}

} // namespace velox::simulation
