#pragma once

#include <cmath>

namespace velox::models {
namespace utils {

namespace detail {
constexpr double kLbToNewton = 4.4482216152605;
constexpr double kFtToMeter  = 0.3048;
constexpr double kLbToKg     = 0.45359237;
constexpr double kGravity    = 9.81;
} // namespace detail

inline double lb_sec2_ft_IN_kg(double prev_val)
{
    // 1lb is 4.4482216152605 N, 1ft is 0.3048 m
    return detail::kLbToNewton / detail::kFtToMeter * prev_val;
}

inline double ft_IN_m(double prev_val)
{
    return detail::kFtToMeter * prev_val;
}

inline double lb_ft_sec2_IN_kg_m2(double prev_val)
{
    // [kg m^2] = [N m sec^2]
    return detail::kLbToNewton * detail::kFtToMeter * prev_val;
}

inline double rad_ft_lb_IN_rad_sec2_kg_m2(double prev_val)
{
    // original: [rad/(ft lb)]
    // new: [rad/(N m)] = [rad s^2/(kg m^2)]
    return 1.0 / (detail::kLbToNewton * detail::kFtToMeter) * prev_val;
}

inline double ft2_IN_m2(double prev_val)
{
    return std::pow(detail::kFtToMeter, 2) * prev_val;
}

inline double lbs_ft_IN_N_m(double prev_val)
{
    // original: [lbs/ft] => [N/m]
    return detail::kLbToKg * detail::kGravity / detail::kFtToMeter * prev_val;
}

inline double lb_sec_ft_IN_N_s_m(double prev_val)
{
    // original: [lb sec/ft] => [N sec/m]
    return detail::kLbToNewton / detail::kFtToMeter * prev_val;
}

inline double ft_lb_rad_IN_N_m_rad(double prev_val)
{
    // original: [lb ft/rad] => [N m/rad]
    return detail::kLbToNewton * detail::kFtToMeter * prev_val;
}

inline double ft_lb_IN_m_N(double prev_val)
{
    // original: [ft/lb] => [m/N]
    return detail::kFtToMeter / detail::kLbToNewton * prev_val;
}

inline double rad_ft_IN_rad_m(double prev_val)
{
    // original: [rad/ft] => [rad/m]
    return 1.0 / detail::kFtToMeter * prev_val;
}

} // namespace utils
} // namespace velox::models
