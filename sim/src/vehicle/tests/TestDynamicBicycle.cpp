#include "DynamicBicycle.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

VehicleParam loadVehicleParams() {
    const std::vector<std::string> candidates = {
        "configs/vehicle/configDry.yaml",
        "../configs/vehicle/configDry.yaml",
        "../../configs/vehicle/configDry.yaml",
        "../../../../configs/vehicle/configDry.yaml"};

    for (const auto& path : candidates) {
        std::ifstream fin(path);
        if (!fin.good()) continue;
        return VehicleParam::loadFromFile(path);
    }
    throw std::runtime_error("Unable to locate vehicle parameter file");
}

void assertFiniteState(const VehicleState& state) {
    assert(std::isfinite(state.velocity.x()));
    assert(std::isfinite(state.velocity.y()));
    assert(std::isfinite(state.rotation.z()));
    assert(std::isfinite(state.yaw));
}

} // namespace

int main() {
    try {
        VehicleParam params = loadVehicleParams();
        DynamicBicycle model(params);

        // --- Low-speed brake + steer: bounded dynamics ---
        {
            VehicleState state;
            state.velocity.x() = 0.8;
            VehicleInput input(-1.0, 0.0, 0.4);
            constexpr double dt = 0.01;
            double max_lat = 0.0;
            double max_yaw = 0.0;
            for (int i = 0; i < 400; ++i) {
                model.updateState(state, input, dt);
                assertFiniteState(state);
                max_lat = std::max(max_lat, std::abs(state.velocity.y()));
                max_yaw = std::max(max_yaw, std::abs(state.rotation.z()));
            }
            assert(max_lat < 3.0);
            assert(max_yaw < 8.0);
        }

        // --- Low-speed with residual yaw: exponential damping ---
        {
            VehicleState state;
            state.velocity.x() = 0.2;
            state.velocity.y() = 0.1;
            state.rotation.z() = 2.0;
            VehicleInput input(-0.7, 0.0, 0.3);
            constexpr double dt = 0.02;
            for (int i = 0; i < 250; ++i) {
                model.updateState(state, input, dt);
                assertFiniteState(state);
            }
            assert(std::abs(state.velocity.y()) < 0.5);
            assert(std::abs(state.rotation.z()) < 1.0);
        }

        // --- Moderate-speed drift capability check ---
        {
            VehicleState state;
            state.velocity.x() = 15.0;
            VehicleInput input(0.4, 0.0, 0.25);
            constexpr double dt = 0.02;
            double slip_sum = 0.0;
            double slip_max = 0.0;
            int slip_samples = 0;
            for (int i = 0; i < 300; ++i) {
                model.updateState(state, input, dt);
                assertFiniteState(state);
                const double vx = state.velocity.x();
                const double vy = state.velocity.y();
                const double speed = std::sqrt(vx * vx + vy * vy);
                if (i > 50 && speed > 1.0) {
                    const double slip = std::atan2(vy, vx) - state.yaw;
                    slip_sum += std::abs(slip);
                    slip_max = std::max(slip_max, std::abs(slip));
                    ++slip_samples;
                }
            }
            assert(slip_samples > 0);
            const double slip_avg = slip_sum / static_cast<double>(slip_samples);
            assert(slip_avg > 0.05);
            assert(slip_max > 0.1);
        }

        std::cout << "DynamicBicycle tests passed" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
