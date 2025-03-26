#ifndef DYNAMICBICYCLE_H
#define DYNAMICBICYCLE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "VehicleModel.h"
#include "State.h"
#include "Input.h"

    // DynamicBicycle structure that “inherits” from VehicleModel.
    typedef struct
    {
        VehicleModel base; // Base vehicle model data and parameters.
    } DynamicBicycle;

    // Initializes a DynamicBicycle instance using a YAML file.
    void DynamicBicycle_init(DynamicBicycle *db, const char *yamlFilePath);

    // Updates the state using the dynamic bicycle model equations.
    void DynamicBicycle_UpdateState(DynamicBicycle *db, State *state, Input *input, double dt);

    // Calculates the magnitude (Euclidean norm) of a 2D vector.
    float DynamicBicycle_CalculateMagnitude(float xComponent, float yComponent);

#ifdef __cplusplus
}
#endif

#endif // DYNAMICBICYCLE_H
