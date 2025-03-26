#include "Input.h"
#include <stdio.h>

Input Input_create(double acceleration, double velocity, double steeringAngle) {
    Input input;
    input.acc = acceleration;
    input.vel = velocity;
    input.delta = steeringAngle;
    return input;
}

void Input_toString(const Input* input, char* buffer, int bufferSize) {
    // Using snprintf to safely format the string into the buffer.
    snprintf(buffer, bufferSize, "acc: %lf | vel: %lf | delta: %lf", input->acc, input->vel, input->delta);
}
