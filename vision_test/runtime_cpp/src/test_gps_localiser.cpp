#include <iostream>
#include <iomanip>
#include <vector>
#include "gps_localiser.hpp"

using namespace GPSLocaliser;
using namespace fsai::control::runtime;

int main() {
    CanIface mock_iface;
    Gps_Local localizer;

    std::cout << "=== GPS Telemetry Logic Test ===" << std::endl;

    struct TestData {
        double lat;
        double lon;
        double speed;
        std::string desc;
    };

    double start_lat = 31.0;
    double start_lon = 121.0;

    std::vector<TestData> steps = {

        {start_lat, start_lon, 0.0, "Step 0: Origin"},

        {start_lat, start_lon + 0.0001, 5.0, "Step 1: Move East"},

        {start_lat + 0.0001, start_lon + 0.0001, 5.0, "Step 2: Move North"},
    };

    for (const auto& step : steps) {
        mock_iface.SetMockData(step.lat, step.lon, step.speed);

        auto result = localizer.Update(mock_iface);

        std::cout << "--- " << step.desc << " ---" << std::endl;
        std::cout << "Input:  " << std::fixed << std::setprecision(5)
                  << step.lat << ", " << step.lon << std::endl;

        if (result) {
            std::cout << "Output: x=" << std::setprecision(2) << result->x_m
                      << " m, y=" << result->y_m
                      << " m, rad=" << result->rad
                      << " (" << (result->rad * 180.0 / M_PI) << " deg)" << std::endl;
        } else {
            std::cout << "Output: [Invalid]" << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}