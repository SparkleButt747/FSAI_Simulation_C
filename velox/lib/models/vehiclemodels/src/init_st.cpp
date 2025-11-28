#include "models/vehiclemodels/init_st.hpp"

#include <algorithm>

namespace velox::models {

std::vector<double> init_st(const std::vector<double>& init_state)
{
    // Ensure we always return a valid 7-state vector for the single-track model.
    // If the caller provided a shorter state (or none), pad with zeros; if longer,
    // truncate to the expected length.
    constexpr std::size_t kStateSize = 7;
    std::vector<double> state(kStateSize, 0.0);
    const auto copy = std::min(init_state.size(), state.size());
    std::copy_n(init_state.begin(), copy, state.begin());
    return state;
}

}  // namespace velox::models
