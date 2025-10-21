#include "sim/integration/provider_registry.hpp"

#include <stdexcept>

namespace fsai {
namespace integration {

void ProviderRegistry::RegisterVision(std::unique_ptr<IVisionProvider> provider) {
    const std::string name = provider->Name();
    vision_[name] = std::move(provider);
}

void ProviderRegistry::RegisterPlanner(std::unique_ptr<IPlannerProvider> provider) {
    const std::string name = provider->Name();
    planner_[name] = std::move(provider);
}

void ProviderRegistry::RegisterEstimator(std::unique_ptr<IEstimatorProvider> provider) {
    const std::string name = provider->Name();
    estimator_[name] = std::move(provider);
}

void ProviderRegistry::RegisterCan(std::unique_ptr<ICanProvider> provider) {
    const std::string name = provider->Name();
    can_[name] = std::move(provider);
}

IVisionProvider* ProviderRegistry::ActivateVision(const std::string& name) {
    return Activate(name, vision_, active_vision_);
}

IPlannerProvider* ProviderRegistry::ActivatePlanner(const std::string& name) {
    return Activate(name, planner_, active_planner_);
}

IEstimatorProvider* ProviderRegistry::ActivateEstimator(const std::string& name) {
    return Activate(name, estimator_, active_estimator_);
}

ICanProvider* ProviderRegistry::ActivateCan(const std::string& name) {
    return Activate(name, can_, active_can_);
}

template <typename ProviderT>
ProviderT* ProviderRegistry::Activate(const std::string& name,
                                      std::unordered_map<std::string, std::unique_ptr<ProviderT>>& store,
                                      ProviderT*& active) {
    auto it = store.find(name);
    if (it == store.end()) {
        throw std::runtime_error("Provider not registered: " + name);
    }
    ProviderT* provider = it->second.get();
    if (provider->Init() != 0) {
        throw std::runtime_error("Provider failed to initialize: " + name);
    }
    active = provider;
    return provider;
}

template IVisionProvider* ProviderRegistry::Activate<IVisionProvider>(
    const std::string&,
    std::unordered_map<std::string, std::unique_ptr<IVisionProvider>>&,
    IVisionProvider*&);

template IPlannerProvider* ProviderRegistry::Activate<IPlannerProvider>(
    const std::string&,
    std::unordered_map<std::string, std::unique_ptr<IPlannerProvider>>&,
    IPlannerProvider*&);

template IEstimatorProvider* ProviderRegistry::Activate<IEstimatorProvider>(
    const std::string&,
    std::unordered_map<std::string, std::unique_ptr<IEstimatorProvider>>&,
    IEstimatorProvider*&);

template ICanProvider* ProviderRegistry::Activate<ICanProvider>(
    const std::string&,
    std::unordered_map<std::string, std::unique_ptr<ICanProvider>>&,
    ICanProvider*&);

}  // namespace integration
}  // namespace fsai

