#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "State.h"
#include "Input.h"
#include "VehicleParam.h" // Defines VehicleParam type and initializer
#include "WheelsInfo.h"   // Defines WheelsInfo type

    // Structure representing the vehicle model.
    typedef struct
    {
        VehicleParam _param;
        // (Optional: function pointers for “virtual” methods could be added here.)
    } VehicleModel;

    // Initializes the VehicleModel using a YAML file path.
    // This will call an initializer in your VehicleParam module.
    void VehicleModel_init(VehicleModel *model, const char *yamlFilePath);

    // Virtual function to update the state (to be overridden in derived models).
    // In the base class, this function does nothing.
    void VehicleModel_UpdateState(VehicleModel *model, State *state, Input *input, double dt);

    // Validates the state (e.g., ensures that v_x is non-negative).
    void VehicleModel_ValidateState(VehicleModel *model, State *state);

    // Validates the input by clamping the values to within allowed ranges.
    void VehicleModel_ValidateInput(VehicleModel *model, Input *input);

    // Computes the slip angle based on the current state and input.
    // Parameter isFront should be non-zero for front axle, zero for rear.
    double VehicleModel_GetSlipAngle(VehicleModel *model, const State *x, const Input *u, int isFront);

    // Computes and returns the wheel speeds.
    WheelsInfo VehicleModel_GetWheelSpeeds(VehicleModel *model, const State *state, const Input *input);

#ifdef __cplusplus
}
#endif

#endif // VEHICLEMODEL_H
