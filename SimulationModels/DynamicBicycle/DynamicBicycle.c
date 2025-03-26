#include "DynamicBicycle.h"
#include "math.h"
#include "stdio.h"

// --- Private Helper Functions ---

// Clamp value between 0 and 1.
static double Clamp01(double value) {
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

// Computes the derivative of the state, analogous to _f in C#.
static State DynamicBicycle_f(DynamicBicycle* db, const State* x, const Input* u, double Fx, double FyF, double FyR) {
    State xDot;
    double FyFTot = 2.0 * FyF;
    double FyRTot = 2.0 * FyR;
    
    xDot.x    = cos(x->yaw) * x->v_x - sin(x->yaw) * x->v_y;
    xDot.y    = sin(x->yaw) * x->v_x + cos(x->yaw) * x->v_y;
    xDot.yaw  = x->r_z;
    
    xDot.v_x  = (x->r_z * x->v_y) + (Fx - sin(u->delta) * FyFTot) / db->base._param.inertia.m;
    xDot.v_y  = ((cos(u->delta) * FyFTot) + FyRTot) / db->base._param.inertia.m - (x->r_z * x->v_x);
    
    xDot.r_z  = (cos(u->delta) * FyFTot * db->base._param.kinematic.l_F - FyRTot * db->base._param.kinematic.l_R)
                  / db->base._param.inertia.I_z;
    
    // Zero out unused fields.
    xDot.z = 0.0;  xDot.v_z = 0.0;
    xDot.r_x = 0.0; xDot.r_y = 0.0;
    xDot.a_x = 0.0; xDot.a_y = 0.0; xDot.a_z = 0.0;
    return xDot;
}

// Applies kinematic correction (analogous to _fKinCorrection in C#).
static State DynamicBicycle_fKinCorrection(DynamicBicycle* db, const State* xIn, const State* xState, const Input* u, double Fx, double dt) {
    State x = *xIn;  // Copy xIn into x.
    double vXDot = Fx / db->base._param.inertia.m;
    double v = DynamicBicycle_CalculateMagnitude((float)xState->v_x, (float)xState->v_y);
    double vBlend = 0.5 * (v - 1.5);
    double blend = Clamp01(vBlend);
    
    x.v_x = blend * x.v_x + (1.0 - blend) * (xState->v_x + dt * vXDot);
    
    double vY = tan(u->delta) * x.v_x * db->base._param.kinematic.l_R / db->base._param.kinematic.l;
    double r  = tan(u->delta) * x.v_x / db->base._param.kinematic.l;
    
    x.v_y = blend * x.v_y + (1.0 - blend) * vY;
    x.r_z = blend * x.r_z + (1.0 - blend) * r;
    return x;
}

// Computes the aerodynamic drag force.
static double DynamicBicycle_GetFdrag(DynamicBicycle* db, const State* x) {
    return db->base._param.aero.c_drag * x->v_x * x->v_x;
}

// Computes the aerodynamic downforce.
static double DynamicBicycle_GetFdown(DynamicBicycle* db, const State* x) {
    return db->base._param.aero.c_down * x->v_x * x->v_x;
}

// Computes the normal force on the vehicle.
static double DynamicBicycle_GetNormalForce(DynamicBicycle* db, const State* x) {
    return db->base._param.inertia.g * db->base._param.inertia.m + DynamicBicycle_GetFdown(db, x);
}

// Computes the longitudinal force.
static double DynamicBicycle_GetFx(DynamicBicycle* db, const State* x, const Input* u) {
    double acc = (x->v_x <= 0.0 && u->acc < 0.0) ? 0.0 : u->acc;
    double Fx = acc * db->base._param.inertia.m - DynamicBicycle_GetFdrag(db, x);
    return Fx;
}

// Returns the front axle downforce.
static double DynamicBicycle_GetDownForceFront(DynamicBicycle* db, double Fz) {
    return 0.5 * db->base._param.kinematic.w_front * Fz;
}

// Returns the rear axle downforce.
static double DynamicBicycle_GetDownForceRear(DynamicBicycle* db, double Fz) {
    return 0.5 * (1.0 - db->base._param.kinematic.w_front) * Fz;
}

// Computes the lateral tire force.
static double DynamicBicycle_GetFy(DynamicBicycle* db, double Fz, int front, double slipAngle) {
    double FzAxle = front ? DynamicBicycle_GetDownForceFront(db, Fz)
                          : DynamicBicycle_GetDownForceRear(db, Fz);
    double B = db->base._param.tire.B;
    double C = db->base._param.tire.C;
    double D = db->base._param.tire.D;
    double E = db->base._param.tire.E;
    double muY = D * sin(C * atan(B * (1.0 - E) * slipAngle + E * atan(B * slipAngle)));
    double Fy = FzAxle * muY;
    return Fy;
}

// --- Public Interface ---

void DynamicBicycle_init(DynamicBicycle* db, const char* yamlFilePath) {
    // Initialize the base VehicleModel with parameters from the YAML file.
    VehicleModel_init(&db->base, yamlFilePath);
}

void DynamicBicycle_UpdateState(DynamicBicycle* db, State* state, Input* input, double dt) {
    // Validate input using the base class function.
    VehicleModel_ValidateInput(&db->base, input);
    
    // Calculate the normal force.
    double Fz = DynamicBicycle_GetNormalForce(db, state);
    
    // Compute slip angles for front and rear.
    double slipAngleFront = VehicleModel_GetSlipAngle(&db->base, state, input, 1); // true for front.
    double FyF = DynamicBicycle_GetFy(db, Fz, 1, slipAngleFront);
    
    double slipAngleBack = VehicleModel_GetSlipAngle(&db->base, state, input, 0);
    double FyR = DynamicBicycle_GetFy(db, Fz, 0, slipAngleBack);
    
    // Drivetrain: compute the longitudinal force.
    double Fx = DynamicBicycle_GetFx(db, state, input);
    
    // Compute dynamics: state derivative and next state.
    State xDotDyn = DynamicBicycle_f(db, state, input, Fx, FyF, FyR);
    State temp = State_multiply(&xDotDyn, dt);
    State xNextDyn = State_add(state, &temp);
    
    // Apply kinematic correction.
    State corrected = DynamicBicycle_fKinCorrection(db, &xNextDyn, state, input, Fx, dt);
    *state = corrected;
    
    // Set acceleration based on the computed derivative.
    state->a_x = xDotDyn.v_x;
    state->a_y = xDotDyn.v_y;
    
    // Validate the updated state.
    VehicleModel_ValidateState(&db->base, state);
}

float DynamicBicycle_CalculateMagnitude(float xComponent, float yComponent) {
    return (float)sqrt(xComponent * xComponent + yComponent * yComponent);
}
