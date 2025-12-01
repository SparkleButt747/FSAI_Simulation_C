#pragma once

#include <optional>
#include <cmath>
#include "can_iface.hpp"
// #include "test_iface.hpp"
// See vision_test/runtime_cpp/src/test_iface.hpp

namespace GPSLocaliser {

    struct Pose {
        double x_m{0.0};
        double y_m{0.0};
        double rad{0.0}; // Facing angle
        bool is_valid{false};
    };

    class Gps_Local {
    public:
        Gps_Local() = default;

        // Read from CAN
        std::optional<Pose> Update(const fsai::control::runtime::CanIface& iface);

        void Reset();

    private:
        // Earth
        static constexpr double kEarthRadiusM = 6371000.0;
        static constexpr double kDegToRad = M_PI / 180.0;

        // A min speed threshold to prevent the facing angle changes when car is idle
        static constexpr double kMovementThresholdMps = 0.5;

        bool initialized_{false};

        double origin_lat_rad_{0.0};
        double origin_lon_rad_{0.0};

        // The previous frame
        double last_x_{0.0};
        double last_y_{0.0};
        double current_heading_{0.0};
    };

}