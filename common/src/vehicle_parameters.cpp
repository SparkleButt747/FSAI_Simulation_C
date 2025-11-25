#include "vehicle_parameters.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace velox::models {

namespace {

// Small helper: assign scalar if YAML key exists
template <typename T>
void assign_if_present(const YAML::Node& node, const char* key, T& target)
{
    if (node[key]) {
        target = node[key].as<T>();
    }
}

// Load nested steering parameters: either from node["steering"] or ignore if not present.
void load_steering(const YAML::Node& root, utils::SteeringParameters& s)
{
    YAML::Node n = root["steering"];
    if (!n || !n.IsMap()) return;

    assign_if_present(n, "min",             s.min);
    assign_if_present(n, "max",             s.max);
    assign_if_present(n, "v_min",           s.v_min);
    assign_if_present(n, "v_max",           s.v_max);
    assign_if_present(n, "kappa_dot_max",   s.kappa_dot_max);
    assign_if_present(n, "kappa_dot_dot_max", s.kappa_dot_dot_max);
}

// Load nested longitudinal parameters: either from node["longitudinal"] or ignore.
void load_longitudinal(const YAML::Node& root, utils::LongitudinalParameters& lp)
{
    YAML::Node n = root["longitudinal"];
    if (!n || !n.IsMap()) return;

    assign_if_present(n, "v_min",    lp.v_min);
    assign_if_present(n, "v_max",    lp.v_max);
    assign_if_present(n, "v_switch", lp.v_switch);
    assign_if_present(n, "a_max",    lp.a_max);
    assign_if_present(n, "j_max",    lp.j_max);
    assign_if_present(n, "j_dot_max", lp.j_dot_max);
}

// Load nested trailer parameters: either from node["trailer"] or ignore.
void load_trailer(const YAML::Node& root, utils::TrailerParameters& tr)
{
    YAML::Node n = root["trailer"];
    if (!n || !n.IsMap()) return;

    assign_if_present(n, "l",      tr.l);
    assign_if_present(n, "w",      tr.w);
    assign_if_present(n, "l_hitch", tr.l_hitch);
    assign_if_present(n, "l_total", tr.l_total);
    assign_if_present(n, "l_wb",   tr.l_wb);
}

// Load tire parameters: the tire YAML may either be a flat mapping with the fields
// or have a top-level "tire" node. We support both.
void load_tire(const YAML::Node& root, utils::TireParameters& tp)
{
    YAML::Node n = root["tire"];
    if (!n || !n.IsMap()) {
        n = root;
    }
    if (!n || !n.IsMap()) return;

    // longitudinal coefficients
    assign_if_present(n, "p_cx1", tp.p_cx1);
    assign_if_present(n, "p_dx1", tp.p_dx1);
    assign_if_present(n, "p_dx3", tp.p_dx3);
    assign_if_present(n, "p_ex1", tp.p_ex1);
    assign_if_present(n, "p_kx1", tp.p_kx1);
    assign_if_present(n, "p_hx1", tp.p_hx1);
    assign_if_present(n, "p_vx1", tp.p_vx1);
    assign_if_present(n, "r_bx1", tp.r_bx1);
    assign_if_present(n, "r_bx2", tp.r_bx2);
    assign_if_present(n, "r_cx1", tp.r_cx1);
    assign_if_present(n, "r_ex1", tp.r_ex1);
    assign_if_present(n, "r_hx1", tp.r_hx1);

    // lateral coefficients
    assign_if_present(n, "p_cy1", tp.p_cy1);
    assign_if_present(n, "p_dy1", tp.p_dy1);
    assign_if_present(n, "p_dy3", tp.p_dy3);
    assign_if_present(n, "p_ey1", tp.p_ey1);
    assign_if_present(n, "p_ky1", tp.p_ky1);
    assign_if_present(n, "p_hy1", tp.p_hy1);
    assign_if_present(n, "p_hy3", tp.p_hy3);
    assign_if_present(n, "p_vy1", tp.p_vy1);
    assign_if_present(n, "p_vy3", tp.p_vy3);
    assign_if_present(n, "r_by1", tp.r_by1);
    assign_if_present(n, "r_by2", tp.r_by2);
    assign_if_present(n, "r_by3", tp.r_by3);
    assign_if_present(n, "r_cy1", tp.r_cy1);
    assign_if_present(n, "r_ey1", tp.r_ey1);
    assign_if_present(n, "r_hy1", tp.r_hy1);
    assign_if_present(n, "r_vy1", tp.r_vy1);
    assign_if_present(n, "r_vy3", tp.r_vy3);
    assign_if_present(n, "r_vy4", tp.r_vy4);
    assign_if_present(n, "r_vy5", tp.r_vy5);
    assign_if_present(n, "r_vy6", tp.r_vy6);
}

// Load all the "plain" vehicle fields (masses, geometry, etc.) from the vehicle YAML.
void load_vehicle_scalars(const YAML::Node& n, VehicleParameters& p)
{
    // vehicle body dimensions
    assign_if_present(n, "l", p.l);
    assign_if_present(n, "w", p.w);

    // masses
    assign_if_present(n, "m",   p.m);
    assign_if_present(n, "m_s", p.m_s);
    assign_if_present(n, "m_uf", p.m_uf);
    assign_if_present(n, "m_ur", p.m_ur);

    // axes distances
    assign_if_present(n, "a", p.a);
    assign_if_present(n, "b", p.b);

    // inertias
    assign_if_present(n, "I_Phi_s", p.I_Phi_s);
    assign_if_present(n, "I_y_s",   p.I_y_s);
    assign_if_present(n, "I_z",     p.I_z);
    assign_if_present(n, "I_xz_s",  p.I_xz_s);

    // suspension parameters
    assign_if_present(n, "K_sf",  p.K_sf);
    assign_if_present(n, "K_sdf", p.K_sdf);
    assign_if_present(n, "K_sr",  p.K_sr);
    assign_if_present(n, "K_sdr", p.K_sdr);

    // geometric parameters
    assign_if_present(n, "T_f",   p.T_f);
    assign_if_present(n, "T_r",   p.T_r);
    assign_if_present(n, "K_ras", p.K_ras);

    assign_if_present(n, "K_tsf", p.K_tsf);
    assign_if_present(n, "K_tsr", p.K_tsr);
    assign_if_present(n, "K_rad", p.K_rad);
    assign_if_present(n, "K_zt",  p.K_zt);

    assign_if_present(n, "h_cg",  p.h_cg);
    assign_if_present(n, "h_raf", p.h_raf);
    assign_if_present(n, "h_rar", p.h_rar);

    assign_if_present(n, "h_s",   p.h_s);

    assign_if_present(n, "I_uf",  p.I_uf);
    assign_if_present(n, "I_ur",  p.I_ur);
    assign_if_present(n, "I_y_w", p.I_y_w);

    assign_if_present(n, "K_lt",  p.K_lt);
    assign_if_present(n, "R_w",   p.R_w);

    // torque split
    assign_if_present(n, "T_sb", p.T_sb);
    assign_if_present(n, "T_se", p.T_se);

    // suspension camber parameters
    assign_if_present(n, "D_f", p.D_f);
    assign_if_present(n, "D_r", p.D_r);
    assign_if_present(n, "E_f", p.E_f);
    assign_if_present(n, "E_r", p.E_r);
}

} // anonymous namespace

VehicleParameters setup_vehicle_parameters(int vehicle_id,
                                           const std::string& dir_params)
{
    // Default param root if none given
#ifndef VELOX_PARAM_ROOT
#define VELOX_PARAM_ROOT "parameters"
#endif

    fs::path root = dir_params.empty()
        ? fs::path(VELOX_PARAM_ROOT)
        : fs::path(dir_params);

    // Vehicle and tire YAML paths
    fs::path vehicle_yaml = root / "vehicle" /
        ("parameters_vehicle" + std::to_string(vehicle_id) + ".yaml");
    fs::path tire_yaml = root / "tire" / "parameters_tire.yaml";

    if (!fs::exists(vehicle_yaml)) {
        throw std::runtime_error("Vehicle parameter file not found: " +
                                 vehicle_yaml.string());
    }
    if (!fs::exists(tire_yaml)) {
        throw std::runtime_error("Tire parameter file not found: " +
                                 tire_yaml.string());
    }

    YAML::Node conf_vehicle = YAML::LoadFile(vehicle_yaml.string());
    YAML::Node conf_tire    = YAML::LoadFile(tire_yaml.string());

    VehicleParameters p;

    // Fill from vehicle YAML
    load_vehicle_scalars(conf_vehicle, p);
    load_steering(conf_vehicle,      p.steering);
    load_longitudinal(conf_vehicle,  p.longitudinal);
    load_trailer(conf_vehicle,       p.trailer);

    // Fill from tire YAML
    load_tire(conf_tire,             p.tire);

    return p;
}

} // namespace velox::models
