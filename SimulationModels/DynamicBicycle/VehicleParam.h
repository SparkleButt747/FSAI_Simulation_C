#ifndef VEHICLEPARAM_H
#define VEHICLEPARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Param.h"  // This header defines Inertia, Kinematic, Tire, Aero, and InputRanges

// Structure representing the vehicle parameters.
typedef struct {
    Inertia inertia;
    Kinematic kinematic;
    Tire tire;
    Aero aero;
    InputRanges input_ranges;
} VehicleParam;

// Initializes the VehicleParam structure by parsing a YAML file.
void VehicleParam_init(VehicleParam* vp, const char* yamlFilePath);

#ifdef __cplusplus
}
#endif

#endif // VEHICLEPARAM_H
