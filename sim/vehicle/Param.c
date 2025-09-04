#include "Param.h"

// A default initializer for Param with all fields set to zero.
// You can modify this function to provide meaningful defaults.
Param Param_default() {
    Param p;
    
    // Inertia defaults.
    p.inertia.m   = 0.0;
    p.inertia.g   = 0.0;
    p.inertia.I_z = 0.0;
    p.inertia.C_f = 0.0;
    p.inertia.C_r = 0.0;
    
    // Kinematic defaults.
    p.kinematic.l         = 0.0;
    p.kinematic.b_F       = 0.0;
    p.kinematic.b_R       = 0.0;
    p.kinematic.w_front   = 0.0;
    p.kinematic.l_F       = 0.0;
    p.kinematic.l_R       = 0.0;
    p.kinematic.axle_width = 0.0;
    
    // Tire defaults.
    p.tire.tire_coefficient = 0.0;
    p.tire.B     = 0.0;
    p.tire.C     = 0.0;
    p.tire.D     = 0.0;
    p.tire.E     = 0.0;
    p.tire.radius = 0.0;
    
    // Aero defaults.
    p.aero.c_down = 0.0;
    p.aero.c_drag = 0.0;
    
    // Input ranges defaults.
    p.input_ranges.acc.min   = 0.0;
    p.input_ranges.acc.max   = 0.0;
    p.input_ranges.vel.min   = 0.0;
    p.input_ranges.vel.max   = 0.0;
    p.input_ranges.delta.min = 0.0;
    p.input_ranges.delta.max = 0.0;
    
    return p;
}
