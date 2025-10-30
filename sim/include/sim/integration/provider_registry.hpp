#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <io/camera/sim_stereo/sim_stereo_source.hpp>

namespace fsai::sim::integration {

using StereoProviderFactory =
    std::function<std::unique_ptr<io::camera::sim_stereo::SimStereoSource>()>;

void registerStereoProvider(const std::string& name, StereoProviderFactory factory);
StereoProviderFactory lookupStereoProvider(const std::string& name);
void registerBuiltInStereoProviders();

}  // namespace fsai::sim::integration
