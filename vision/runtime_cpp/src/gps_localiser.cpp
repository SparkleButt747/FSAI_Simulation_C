#include "gps_localiser.hpp"

namespace GPSLocaliser {

    void Gps_Local::Reset() {
        initialized_ = false;
        origin_lat_rad_ = 0.0;
        origin_lon_rad_ = 0.0;
        last_x_ = 0.0;
        last_y_ = 0.0;
        current_heading_ = 0.0;
    }

    std::optional<Pose> Gps_Local::Update(const fsai::control::runtime::CanIface& iface) {
        // Read GPS data
        auto gps_opt = iface.LatestGps();
        if (!gps_opt) {
            return std::nullopt;
        }

        const auto& gps = *gps_opt;

        // Degree 2 Radian
        double lat_rad = gps.lat_deg * kDegToRad;
        double lon_rad = gps.lon_deg * kDegToRad;

        // Init origin with the first frame
        if (!initialized_) {
            origin_lat_rad_ = lat_rad;
            origin_lon_rad_ = lon_rad;
            initialized_ = true;

            // Reset pos
            last_x_ = 0.0;
            last_y_ = 0.0;
            current_heading_ = 0.0;

            return Pose{0.0, 0.0, 0.0, true};
        }

        // Flat earth approximation
        double d_lat = lat_rad - origin_lat_rad_;
        double d_lon = lon_rad - origin_lon_rad_;

        // Use mean latitude to reduce error
        double mean_lat = (lat_rad + origin_lat_rad_) / 2.0;

        // Y -> North
        double y = kEarthRadiusM * d_lat;

        // X --> East
        double x = kEarthRadiusM * d_lon * std::cos(mean_lat);

        // Facing angle
        double speed = gps.speed_mps;

        if (speed > kMovementThresholdMps) {
            double dx = x - last_x_;
            double dy = y - last_y_;

            if (std::abs(dx) > 1e-3 || std::abs(dy) > 1e-3) {
                current_heading_ = std::atan2(dy, dx);
            }
        }

        // Update history position
        last_x_ = x;
        last_y_ = y;

        return Pose{x, y, current_heading_, true};
    }

}