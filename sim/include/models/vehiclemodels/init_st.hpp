#pragma once

#include <vector>

namespace velox::models {

/**
 * init_st - generates the initial state vector for the single-track model.
 *
 * This is a pass-through of the core initial states.
 */
std::vector<double> init_st(const std::vector<double>& init_state);

}  // namespace velox::models
