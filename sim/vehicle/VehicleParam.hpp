#pragma once
#include <string>
#include <yaml.h>
#include <cstdlib>
#include <cstring>

struct Inertia {
    double m{0.0};
    double g{0.0};
    double I_z{0.0};
    double C_f{0.0};
    double C_r{0.0};
};

struct Kinematic {
    double l{0.0};
    double b_F{0.0};
    double b_R{0.0};
    double w_front{0.0};
    double l_F{0.0};
    double l_R{0.0};
    double axle_width{0.0};
};

struct Tire {
    double tire_coefficient{0.0};
    double B{0.0};
    double C{0.0};
    double D{0.0};
    double E{0.0};
    double radius{0.0};
};

struct Aero {
    double c_down{0.0};
    double c_drag{0.0};
};

struct Range { double min{0.0}; double max{0.0}; };

struct InputRanges {
    Range acc;
    Range vel;
    Range delta;
};

/**
 * @brief Parameter container for the vehicle. Values are loaded from a YAML file.
 */
class VehicleParam {
public:
    Inertia inertia;
    Kinematic kinematic;
    Tire tire;
    Aero aero;
    InputRanges input_ranges;

    static VehicleParam loadFromFile(const std::string& yamlFile) {
        VehicleParam vp;
        FILE* fh = std::fopen(yamlFile.c_str(), "rb");
        if (!fh) {
            throw std::runtime_error("Failed to open YAML file: " + yamlFile);
        }
        yaml_parser_t parser; yaml_event_t event;
        if (!yaml_parser_initialize(&parser)) {
            std::fclose(fh);
            throw std::runtime_error("Failed to initialise YAML parser");
        }
        yaml_parser_set_input_file(&parser, fh);

        enum Section { NONE, INERTIA, KINEMATICS, TIRE, AERO, INPUT_RANGES, ACCELERATION, VELOCITY, STEERING } section = NONE;
        std::string lastKey;
        bool done = false;
        while (!done) {
            if (!yaml_parser_parse(&parser, &event)) {
                yaml_parser_delete(&parser); std::fclose(fh);
                throw std::runtime_error("YAML parse error");
            }
            switch (event.type) {
            case YAML_SCALAR_EVENT: {
                std::string scalar = reinterpret_cast<char*>(event.data.scalar.value);
                if (lastKey.empty()) {
                    if (scalar == "inertia") section = INERTIA;
                    else if (scalar == "kinematics") section = KINEMATICS;
                    else if (scalar == "tire") section = TIRE;
                    else if (scalar == "aero") section = AERO;
                    else if (scalar == "input_ranges") section = INPUT_RANGES;
                    else if (section == INPUT_RANGES && scalar == "acceleration") section = ACCELERATION;
                    else if (section == INPUT_RANGES && scalar == "velocity") section = VELOCITY;
                    else if (section == INPUT_RANGES && scalar == "steering") section = STEERING;
                    else lastKey = scalar;
                } else {
                    double value = std::atof(scalar.c_str());
                    switch (section) {
                    case INERTIA:
                        if (lastKey == "m") vp.inertia.m = value;
                        else if (lastKey == "g") vp.inertia.g = value;
                        else if (lastKey == "I_z") vp.inertia.I_z = value;
                        else if (lastKey == "Cf") vp.inertia.C_f = value;
                        else if (lastKey == "Cr") vp.inertia.C_r = value;
                        break;
                    case KINEMATICS:
                        if (lastKey == "l") vp.kinematic.l = value;
                        else if (lastKey == "b_F") vp.kinematic.b_F = value;
                        else if (lastKey == "b_R") vp.kinematic.b_R = value;
                        else if (lastKey == "w_front") vp.kinematic.w_front = value;
                        else if (lastKey == "axle_width") vp.kinematic.axle_width = value;
                        break;
                    case TIRE:
                        if (lastKey == "tire_coefficient") vp.tire.tire_coefficient = value;
                        else if (lastKey == "B") vp.tire.B = value / vp.tire.tire_coefficient;
                        else if (lastKey == "C") vp.tire.C = value;
                        else if (lastKey == "D") vp.tire.D = value * vp.tire.tire_coefficient;
                        else if (lastKey == "E") vp.tire.E = value;
                        else if (lastKey == "radius") vp.tire.radius = value;
                        break;
                    case AERO:
                        if (lastKey == "C_Down") vp.aero.c_down = value;
                        else if (lastKey == "C_drag") vp.aero.c_drag = value;
                        break;
                    case ACCELERATION:
                        if (lastKey == "min") vp.input_ranges.acc.min = value;
                        else if (lastKey == "max") vp.input_ranges.acc.max = value;
                        break;
                    case VELOCITY:
                        if (lastKey == "min") vp.input_ranges.vel.min = value;
                        else if (lastKey == "max") vp.input_ranges.vel.max = value;
                        break;
                    case STEERING:
                        if (lastKey == "min") vp.input_ranges.delta.min = value;
                        else if (lastKey == "max") vp.input_ranges.delta.max = value;
                        break;
                    default: break;
                    }
                    lastKey.clear();
                }
                break; }
            case YAML_MAPPING_END_EVENT:
                if (section==ACCELERATION||section==VELOCITY||section==STEERING) section = INPUT_RANGES;
                break;
            case YAML_STREAM_END_EVENT:
                done = true; break;
            default: break;
            }
            yaml_event_delete(&event);
        }
        yaml_parser_delete(&parser);
        std::fclose(fh);

        vp.kinematic.l_F = vp.kinematic.l * (1.0 - vp.kinematic.w_front);
        vp.kinematic.l_R = vp.kinematic.l * vp.kinematic.w_front;
        return vp;
    }
};
