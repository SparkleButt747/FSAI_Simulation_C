#pragma once
#include <optional>

namespace fsai::control::runtime {

    struct GpsSample {
        double lat_deg{0.0};
        double lon_deg{0.0};
        double speed_mps{0.0};
    };

    class CanIface {
    public:

        std::optional<GpsSample> LatestGps() const {
            if (!has_data_) return std::nullopt;
            return data_;
        }

        void SetMockData(double lat, double lon, double speed) {
            data_.lat_deg = lat;
            data_.lon_deg = lon;
            data_.speed_mps = speed;
            has_data_ = true;
        }

    private:
        GpsSample data_{};
        bool has_data_{false};
    };

}