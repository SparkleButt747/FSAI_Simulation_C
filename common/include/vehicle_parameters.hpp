#pragma once

#include <string>

#include "models/longitudinal_parameters.hpp"
#include "models/steering_parameters.hpp"
#include "models/tireParameters.hpp"
#include "models/trailer_parameters.hpp"

namespace velox::models {

/**
 * VehicleParameters base class: defines all parameters used by the vehicle models
 * described in:
 *
 *   Althoff, M. and Würsching, G. "CommonRoad: Vehicle Models", 2020
 *
 * This is a direct C++ analogue of the Python dataclass.
 */
struct VehicleParameters {
    // vehicle body dimensions
    double l{};  // length [m]
    double w{};  // width [m]

    // steering parameters
    utils::SteeringParameters steering{};

    // longitudinal parameters
    utils::LongitudinalParameters longitudinal{};

    // masses
    double m{};    // total mass
    double m_s{};  // sprung mass
    double m_uf{}; // unsprung mass front
    double m_ur{}; // unsprung mass rear

    // axes distances
    double a{};  // distance from sprung-mass CoG to front axle [m]
    double b{};  // distance from sprung-mass CoG to rear axle [m]

    // moments of inertia of sprung mass
    double I_Phi_s{}; // roll inertia [kg m^2]
    double I_y_s{};   // pitch inertia [kg m^2]
    double I_z{};     // yaw inertia [kg m^2]
    double I_xz_s{};  // roll–yaw product of inertia [kg m^2]

    // suspension parameters
    double K_sf{};  // suspension spring rate (front) [N/m]
    double K_sdf{}; // suspension damping rate (front) [N s/m]
    double K_sr{};  // suspension spring rate (rear) [N/m]
    double K_sdr{}; // suspension damping rate (rear) [N s/m]

    // geometric parameters
    double T_f{};   // track width front [m]
    double T_r{};   // track width rear [m]
    double K_ras{}; // lateral spring rate at compliant pin joint between M_s and M_u [N/m]

    double K_tsf{}; // auxiliary torsion roll stiffness per axle (front) [N m/rad]
    double K_tsr{}; // auxiliary torsion roll stiffness per axle (rear) [N m/rad]
    double K_rad{}; // damping rate at compliant pin joint between M_s and M_u [N s/m]
    double K_zt{};  // vertical spring rate of tire [N/m]

    double h_cg{};  // CoG height of total mass [m]
    double h_raf{}; // height of roll axis above ground (front) [m]
    double h_rar{}; // height of roll axis above ground (rear) [m]

    double h_s{};   // sprung-mass CoG height above ground [m]

    double I_uf{};  // unsprung-mass inertia about x-axis (front) [kg m^2]
    double I_ur{};  // unsprung-mass inertia about x-axis (rear) [kg m^2]
    double I_y_w{}; // wheel inertia [kg m^2]

    double K_lt{};  // lateral compliance rate of tire/wheel/suspension per tire [m/N]
    double R_w{};   // effective wheel/tire radius [m]

    // split of brake and engine torque
    double T_sb{};  // front axle brake torque split [0..1]
    double T_se{};  // front axle engine torque split [0..1]

    // suspension camber parameters
    double D_f{};   // [rad/m]
    double D_r{};   // [rad/m]
    double E_f{};   // [dimensionless, may need conversion]
    double E_r{};   // [dimensionless, may need conversion]

    // tire parameters
    utils::TireParameters tire{};

    // trailer parameters (for kst model)
    utils::TrailerParameters trailer{};
};

/**
 * setup_vehicle_parameters
 *
 * Creates a VehicleParameters object holding all vehicle parameters for a given vehicle type ID.
 * Parameters are read from YAML files in a parameter directory.
 *
 * @param vehicle_id  CommonRoad vehicle ID (1..4 as in the reference paper)
 * @param dir_params  Optional path to the parameter directory containing subfolders
 *                    "vehicle/" and "tire/". If empty, a compiled-in default is used
 *                    (typically "parameters").
 *
 * @return VehicleParameters object populated from YAML.
 *
 * Throws std::runtime_error if required files are missing or cannot be parsed.
 */
VehicleParameters setup_vehicle_parameters(int vehicle_id,
                                           const std::string& dir_params = {});

} // namespace velox::models
