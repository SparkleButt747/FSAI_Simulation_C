#include "sim/integration/runtime_config.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace fsai {
namespace integration {

namespace {

float ReadScalar(const YAML::Node& node, const char* key, float def) {
    if (!node || !node[key]) {
        return def;
    }
    return node[key].as<float>();
}

std::string ReadString(const YAML::Node& node, const char* key, const std::string& def) {
    if (!node || !node[key]) {
        return def;
    }
    return node[key].as<std::string>();
}

}  // namespace

RuntimeConfig LoadRuntimeConfig(const std::string& path) {
    RuntimeConfig cfg;

    try {
        YAML::Node root = YAML::LoadFile(path);

        const YAML::Node providers = root["providers"];
        cfg.providers.vision = ReadString(providers, "vision", cfg.providers.vision);
        cfg.providers.planner = ReadString(providers, "planner", cfg.providers.planner);
        cfg.providers.estimator = ReadString(providers, "estimator", cfg.providers.estimator);
        cfg.providers.can = ReadString(providers, "can", cfg.providers.can);
        cfg.providers.mode = ReadString(providers, "mode", cfg.providers.mode);

        const YAML::Node fake_vision = root["fake_vision"];
        cfg.fake_vision.position_noise_std = std::max(0.0f, ReadScalar(fake_vision, "position_noise_std", cfg.fake_vision.position_noise_std));
        cfg.fake_vision.dropout_probability = std::clamp(ReadScalar(fake_vision, "dropout_probability", cfg.fake_vision.dropout_probability), 0.0f, 1.0f);
        cfg.fake_vision.min_conf = std::clamp(ReadScalar(fake_vision, "min_conf", cfg.fake_vision.min_conf), 0.0f, 1.0f);
        cfg.fake_vision.max_conf = std::clamp(ReadScalar(fake_vision, "max_conf", cfg.fake_vision.max_conf), 0.0f, 1.0f);
        if (cfg.fake_vision.max_conf < cfg.fake_vision.min_conf) {
            cfg.fake_vision.max_conf = cfg.fake_vision.min_conf;
        }

        const YAML::Node fake_planner = root["fake_planner"];
        cfg.fake_planner.sample_spacing_m = std::max(0.1f, ReadScalar(fake_planner, "sample_spacing_m", cfg.fake_planner.sample_spacing_m));
        cfg.fake_planner.horizon_m = std::max(cfg.fake_planner.sample_spacing_m,
                                              ReadScalar(fake_planner, "horizon_m", cfg.fake_planner.horizon_m));
    } catch (const std::exception& ex) {
        throw std::runtime_error(std::string("Failed to load runtime config from ") + path + ": " + ex.what());
    }

    return cfg;
}

}  // namespace integration
}  // namespace fsai

