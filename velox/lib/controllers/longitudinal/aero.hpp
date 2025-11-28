#pragma once

namespace velox::controllers::longitudinal {

struct AeroConfig {
    double drag_coefficient           = 0.0;
    double downforce_coefficient      = 0.0;

    void validate() const;
};

class AeroModel {
public:
    explicit AeroModel(AeroConfig config);

    [[nodiscard]] double drag_force(double speed) const noexcept;

    [[nodiscard]] double downforce(double speed) const noexcept;

    [[nodiscard]] const AeroConfig& config() const noexcept { return config_; }

private:
    AeroConfig config_;
};

} // namespace velox::controllers::longitudinal
