#ifndef INPUT_H
#define INPUT_H

#ifdef __cplusplus
extern "C"
{
#endif

    // The Input struct holds three values: acceleration, velocity, and steering angle.
    typedef struct
    {
        double acc;
        double vel;
        double delta;
    } Input;

    // "Constructor" function to create and initialize an Input struct.
    Input Input_create(double acceleration, double velocity, double steeringAngle);

    // Function to convert an Input struct to a string.
    // The string is written into the provided buffer, which should have at least bufferSize bytes.
    void Input_toString(const Input *input, char *buffer, int bufferSize);

#ifdef __cplusplus
}
#endif

#endif // INPUT_H
