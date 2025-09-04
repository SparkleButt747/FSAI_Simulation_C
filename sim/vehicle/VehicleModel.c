#include "VehicleModel.h"
#include "math.h"  // For M_PI, fmax, atan

// Helper function: clamp a value between min and max.
static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Initializes the VehicleModel, including parsing parameters from the YAML file.
// (Assumes VehicleParam_init is implemented in your VehicleParam module.)
void VehicleModel_init(VehicleModel* model, const char* yamlFilePath) {
    VehicleParam_init(&model->_param, yamlFilePath);
}

// Virtual function: Base UpdateState does nothing.
// Derived models (e.g., DynamicBicycle) should implement their own version.
void VehicleModel_UpdateState(VehicleModel* model, State* state, Input* input, double dt) {
    (void)model;
    (void)state;
    (void)input;
    (void)dt;
    // Base class does not update state.
}

// Ensures that the longitudinal velocity is non-negative.
void VehicleModel_ValidateState(VehicleModel* model, State* state) {
    (void)model; // Not used in this function.
    state->v_x = fmax(0.0, state->v_x);
}

// Clamps the input fields to the allowed ranges as defined in the parameters.
void VehicleModel_ValidateInput(VehicleModel* model, Input* input) {
    double maxAcc   = model->_param.input_ranges.acc.max;
    double minAcc   = model->_param.input_ranges.acc.min;
    double maxVel   = model->_param.input_ranges.vel.max;
    double minVel   = model->_param.input_ranges.vel.min;
    double maxDelta = model->_param.input_ranges.delta.max;
    double minDelta = model->_param.input_ranges.delta.min;
    
    input->acc   = clamp(input->acc,   minAcc,   maxAcc);
    input->vel   = clamp(input->vel,   minVel,   maxVel);
    input->delta = clamp(input->delta, minDelta, maxDelta);
}

// Computes the slip angle using the vehicle state and input.
// For rear (isFront==0): slip angle = atan((v_y - leverArm*r_z) / (max(1.0, v_x) - 0.5*axle_width*r_z)).
// For front (isFront nonzero): subtract input.delta after computing the similar value.
double VehicleModel_GetSlipAngle(VehicleModel* model, const State* x, const Input* u, int isFront) {
    double leverArmLength = model->_param.kinematic.l * model->_param.kinematic.w_front;
    if (!isFront) {
        double vX = fmax(1.0, x->v_x);
        return atan((x->v_y - leverArmLength * x->r_z) / (vX - 0.5 * model->_param.kinematic.axle_width * x->r_z));
    } else {
        double vXFront = fmax(1.0, x->v_x);
        return atan((x->v_y + leverArmLength * x->r_z) / (vXFront - 0.5 * model->_param.kinematic.axle_width * x->r_z)) - u->delta;
    }
}

// Computes the wheel speeds, assuming all wheels have the same speed.
// The wheel speed (in RPM) is computed from the vehicle's v_x and tire radius.
WheelsInfo VehicleModel_GetWheelSpeeds(VehicleModel* model, const State* state, const Input* input) {
    WheelsInfo wheelSpeeds;
    double wheelCircumference = 2.0 * M_PI * model->_param.tire.radius;
    
    wheelSpeeds.steering = input->delta;
    
    // Compute wheel speed in RPM.
    double rpm = (state->v_x / wheelCircumference) * 60.0;
    wheelSpeeds.lf_speed = rpm;
    wheelSpeeds.rf_speed = rpm;
    wheelSpeeds.lb_speed = rpm;
    wheelSpeeds.rb_speed = rpm;
    
    return wheelSpeeds;
}
