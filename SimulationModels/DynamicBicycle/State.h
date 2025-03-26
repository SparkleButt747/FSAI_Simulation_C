#ifndef STATE_H
#define STATE_H

#ifdef __cplusplus
extern "C"
{
#endif

    // The State struct holds position, orientation, velocities, rotations, and accelerations.
    typedef struct
    {
        double x;
        double y;
        double z;
        double yaw;
        double v_x;
        double v_y;
        double v_z;
        double r_x;
        double r_y;
        double r_z;
        double a_x;
        double a_y;
        double a_z;
    } State;

    // "Constructor": Creates a State with specified values.
    State State_create(double x, double y, double z, double yaw,
                       double v_x, double v_y, double v_z,
                       double r_x, double r_y, double r_z,
                       double a_x, double a_y, double a_z);

    // Convenience function to create a State with all fields set to zero.
    State State_default();

    // Multiplies every field of the state by dt (equivalent to operator* in C#).
    State State_multiply(const State *state, double dt);

    // Adds two States component-wise (equivalent to operator+ in C#).
    State State_add(const State *state1, const State *state2);

    // Writes a string representation of the State into the provided buffer.
    // The caller must supply a buffer of at least bufferSize bytes.
    void State_toString(const State *state, char *buffer, int bufferSize);

#ifdef __cplusplus
}
#endif

#endif // STATE_H
