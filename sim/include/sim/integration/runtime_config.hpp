#pragma once

#include <string>

namespace fsai {
namespace integration {

struct ProviderSelection {
    std::string vision{"fake_from_sim"};
    std::string planner{"fake_track"};
    std::string estimator{"fake_truth"};
    std::string can{"fake"};
    std::string mode{"sim"};
};

struct FakeVisionOptions {
    float position_noise_std{0.05f};
    float dropout_probability{0.08f};
    float min_conf{0.80f};
    float max_conf{0.95f};
};

struct FakePlannerOptions {
    float sample_spacing_m{0.75f};
    float horizon_m{80.0f};
};

struct RuntimeConfig {
    ProviderSelection providers{};
    FakeVisionOptions fake_vision{};
    FakePlannerOptions fake_planner{};
};

RuntimeConfig LoadRuntimeConfig(const std::string& path);

}  // namespace integration
}  // namespace fsai

