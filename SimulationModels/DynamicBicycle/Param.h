#ifndef PARAM_H
#define PARAM_H

#ifdef __cplusplus
extern "C"
{
#endif

    // Inertia parameters.
    typedef struct
    {
        double m;
        double g;
        double I_z;
        double C_f;
        double C_r;
    } Inertia;

    // Kinematic parameters.
    typedef struct
    {
        double l;
        double b_F;
        double b_R;
        double w_front;
        double l_F;
        double l_R;
        double axle_width;
    } Kinematic;

    // Tire parameters.
    typedef struct
    {
        double tire_coefficient;
        double B;
        double C;
        double D;
        double E;
        double radius;
    } Tire;

    // Aerodynamic parameters.
    typedef struct
    {
        double c_down;
        double c_drag;
    } Aero;

    // A simple range with minimum and maximum values.
    typedef struct
    {
        double min;
        double max;
    } Range;

    // Input ranges for various controls.
    typedef struct
    {
        Range acc;
        Range vel;
        Range delta;
    } InputRanges;

    // Top-level parameter structure.
    typedef struct
    {
        Inertia inertia;
        Kinematic kinematic;
        Tire tire;
        Aero aero;
        InputRanges input_ranges;
    } Param;

    // Optional: A function to initialize a default Param.
    // (Customize this initializer with nonzero defaults as needed.)
    Param Param_default();

#ifdef __cplusplus
}
#endif

#endif // PARAM_H
