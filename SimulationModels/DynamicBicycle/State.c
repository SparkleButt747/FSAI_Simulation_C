#include "State.h"
#include <stdio.h>

State State_create(double x, double y, double z, double yaw,
                   double v_x, double v_y, double v_z,
                   double r_x, double r_y, double r_z,
                   double a_x, double a_y, double a_z)
{
    State s;
    s.x = x;
    s.y = y;
    s.z = z;
    s.yaw = yaw;
    s.v_x = v_x;
    s.v_y = v_y;
    s.v_z = v_z;
    s.r_x = r_x;
    s.r_y = r_y;
    s.r_z = r_z;
    s.a_x = a_x;
    s.a_y = a_y;
    s.a_z = a_z;
    return s;
}

State State_default() {
    return State_create(0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0);
}

State State_multiply(const State* state, double dt) {
    State result;
    result.x    = state->x    * dt;
    result.y    = state->y    * dt;
    result.z    = state->z    * dt;
    result.yaw  = state->yaw  * dt;
    result.v_x  = state->v_x  * dt;
    result.v_y  = state->v_y  * dt;
    result.v_z  = state->v_z  * dt;
    result.r_x  = state->r_x  * dt;
    result.r_y  = state->r_y  * dt;
    result.r_z  = state->r_z  * dt;
    result.a_x  = state->a_x  * dt;
    result.a_y  = state->a_y  * dt;
    result.a_z  = state->a_z  * dt;
    return result;
}

State State_add(const State* s1, const State* s2) {
    State result;
    result.x    = s1->x    + s2->x;
    result.y    = s1->y    + s2->y;
    result.z    = s1->z    + s2->z;
    result.yaw  = s1->yaw  + s2->yaw;
    result.v_x  = s1->v_x  + s2->v_x;
    result.v_y  = s1->v_y  + s2->v_y;
    result.v_z  = s1->v_z  + s2->v_z;
    result.r_x  = s1->r_x  + s2->r_x;
    result.r_y  = s1->r_y  + s2->r_y;
    result.r_z  = s1->r_z  + s2->r_z;
    result.a_x  = s1->a_x  + s2->a_x;
    result.a_y  = s1->a_y  + s2->a_y;
    result.a_z  = s1->a_z  + s2->a_z;
    return result;
}

void State_toString(const State* s, char* buffer, int bufferSize) {
    snprintf(buffer, bufferSize,
             "x:%lf | y:%lf | z:%lf | yaw:%lf | v_x:%lf | v_y:%lf | v_z:%lf | r_x:%lf | r_y:%lf | r_z:%lf | a_x:%lf | a_y:%lf | a_z:%lf",
             s->x, s->y, s->z, s->yaw,
             s->v_x, s->v_y, s->v_z,
             s->r_x, s->r_y, s->r_z,
             s->a_x, s->a_y, s->a_z);
}
