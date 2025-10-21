#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "sim/integration/providers.hpp"

namespace fsai {
namespace integration {

class ProviderRegistry {
public:
    void RegisterVision(std::unique_ptr<IVisionProvider> provider);
    void RegisterPlanner(std::unique_ptr<IPlannerProvider> provider);
    void RegisterEstimator(std::unique_ptr<IEstimatorProvider> provider);
    void RegisterCan(std::unique_ptr<ICanProvider> provider);

    IVisionProvider* ActivateVision(const std::string& name);
    IPlannerProvider* ActivatePlanner(const std::string& name);
    IEstimatorProvider* ActivateEstimator(const std::string& name);
    ICanProvider* ActivateCan(const std::string& name);

    IVisionProvider* ActiveVision() const { return active_vision_; }
    IPlannerProvider* ActivePlanner() const { return active_planner_; }
    IEstimatorProvider* ActiveEstimator() const { return active_estimator_; }
    ICanProvider* ActiveCan() const { return active_can_; }

private:
    template <typename ProviderT>
    ProviderT* Activate(const std::string& name,
                        std::unordered_map<std::string, std::unique_ptr<ProviderT>>& store,
                        ProviderT*& active);

    std::unordered_map<std::string, std::unique_ptr<IVisionProvider>> vision_{};
    std::unordered_map<std::string, std::unique_ptr<IPlannerProvider>> planner_{};
    std::unordered_map<std::string, std::unique_ptr<IEstimatorProvider>> estimator_{};
    std::unordered_map<std::string, std::unique_ptr<ICanProvider>> can_{};

    IVisionProvider* active_vision_{nullptr};
    IPlannerProvider* active_planner_{nullptr};
    IEstimatorProvider* active_estimator_{nullptr};
    ICanProvider* active_can_{nullptr};
};

}  // namespace integration
}  // namespace fsai

