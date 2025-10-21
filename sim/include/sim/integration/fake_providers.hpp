#pragma once

#include <random>

#include "sim/integration/providers.hpp"
#include "sim/integration/runtime_config.hpp"

namespace fsai {
namespace integration {

std::unique_ptr<IVisionProvider> MakeFakeVisionProvider(const PathTruth& truth, const FakeVisionOptions& options);
std::unique_ptr<IPlannerProvider> MakeFakePlannerProvider(const PathTruth& truth, const FakePlannerOptions& options);
std::unique_ptr<IEstimatorProvider> MakeFakeEstimatorProvider();
std::unique_ptr<ICanProvider> MakeFakeCanProvider();

}  // namespace integration
}  // namespace fsai

