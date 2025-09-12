#pragma once
#include <string>
#include <cstdio>

/**
 * @brief Represents driver input to the vehicle.
 */
class VehicleInput {
public:
    double acc{0.0};   ///< Acceleration command
    double vel{0.0};   ///< Desired velocity (unused in dynamic model but kept for completeness)
    double delta{0.0}; ///< Steering angle

    VehicleInput() = default;
    VehicleInput(double acceleration, double velocity, double steering)
        : acc(acceleration), vel(velocity), delta(steering) {}

    std::string toString() const {
        char buffer[128];
        std::snprintf(buffer, sizeof(buffer), "acc:%f vel:%f delta:%f", acc, vel, delta);
        return std::string(buffer);
    }
};
