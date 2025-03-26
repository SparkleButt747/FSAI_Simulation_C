#include "VehicleParam.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <yaml.h>

// Helper function: compare two strings (case-sensitive)
static int equals(const char* a, const char* b) {
    return strcmp(a, b) == 0;
}

void VehicleParam_init(VehicleParam* vp, const char* yamlFilePath) {
    FILE* fh = fopen(yamlFilePath, "rb");
    if (!fh) {
        fprintf(stderr, "Error opening YAML file: %s\n", yamlFilePath);
        exit(EXIT_FAILURE);
    }

    yaml_parser_t parser;
    yaml_event_t event;
    
    if (!yaml_parser_initialize(&parser)) {
        fprintf(stderr, "Failed to initialize YAML parser\n");
        fclose(fh);
        exit(EXIT_FAILURE);
    }
    yaml_parser_set_input_file(&parser, fh);

    // Define an enum for tracking which section we are parsing.
    enum Section {
        NONE,
        INERTIA,
        KINEMATICS,
        TIRE,
        AERO,
        INPUT_RANGES,
        ACCELERATION,
        VELOCITY,
        STEERING
    } section = NONE;
    
    char* lastKey = NULL;
    int done = 0;
    
    while (!done) {
        if (!yaml_parser_parse(&parser, &event)) {
            fprintf(stderr, "YAML parse error: %s\n", parser.problem);
            exit(EXIT_FAILURE);
        }
        
        switch (event.type) {
            case YAML_SCALAR_EVENT: {
                char* scalar = (char*)event.data.scalar.value;
                // If lastKey is NULL, then this scalar may be a key.
                if (lastKey == NULL) {
                    // Check if this scalar is a section header.
                    if (equals(scalar, "inertia")) {
                        section = INERTIA;
                    } else if (equals(scalar, "kinematics")) {
                        section = KINEMATICS;
                    } else if (equals(scalar, "tire")) {
                        section = TIRE;
                    } else if (equals(scalar, "aero")) {
                        section = AERO;
                    } else if (equals(scalar, "input_ranges")) {
                        section = INPUT_RANGES;
                    }
                    else if (section == INPUT_RANGES &&
                             (equals(scalar, "acceleration") || equals(scalar, "velocity") || equals(scalar, "steering"))) {
                        if (equals(scalar, "acceleration"))
                            section = ACCELERATION;
                        else if (equals(scalar, "velocity"))
                            section = VELOCITY;
                        else if (equals(scalar, "steering"))
                            section = STEERING;
                    }
                    else {
                        // Otherwise, treat this scalar as a key.
                        lastKey = strdup(scalar);
                    }
                } else {
                    // A value corresponding to the last key.
                    if (section == INERTIA) {
                        if (equals(lastKey, "m"))
                            vp->inertia.m = atof(scalar);
                        else if (equals(lastKey, "g"))
                            vp->inertia.g = atof(scalar);
                        else if (equals(lastKey, "I_z"))
                            vp->inertia.I_z = atof(scalar);
                        else if (equals(lastKey, "Cf"))
                            vp->inertia.C_f = atof(scalar);
                        else if (equals(lastKey, "Cr"))
                            vp->inertia.C_r = atof(scalar);
                    }
                    else if (section == KINEMATICS) {
                        if (equals(lastKey, "l"))
                            vp->kinematic.l = atof(scalar);
                        else if (equals(lastKey, "b_F"))
                            vp->kinematic.b_F = atof(scalar);
                        else if (equals(lastKey, "b_R"))
                            vp->kinematic.b_R = atof(scalar);
                        else if (equals(lastKey, "w_front"))
                            vp->kinematic.w_front = atof(scalar);
                        else if (equals(lastKey, "axle_width"))
                            vp->kinematic.axle_width = atof(scalar);
                    }
                    else if (section == TIRE) {
                        if (equals(lastKey, "tire_coefficient"))
                            vp->tire.tire_coefficient = atof(scalar);
                        else if (equals(lastKey, "B"))
                            vp->tire.B = atof(scalar) / vp->tire.tire_coefficient;
                        else if (equals(lastKey, "C"))
                            vp->tire.C = atof(scalar);
                        else if (equals(lastKey, "D"))
                            vp->tire.D = atof(scalar) * vp->tire.tire_coefficient;
                        else if (equals(lastKey, "E"))
                            vp->tire.E = atof(scalar);
                        else if (equals(lastKey, "radius"))
                            vp->tire.radius = atof(scalar);
                    }
                    else if (section == AERO) {
                        if (equals(lastKey, "C_Down"))
                            vp->aero.c_down = atof(scalar);
                        else if (equals(lastKey, "C_drag"))
                            vp->aero.c_drag = atof(scalar);
                    }
                    else if (section == ACCELERATION) {
                        if (equals(lastKey, "min"))
                            vp->input_ranges.acc.min = atof(scalar);
                        else if (equals(lastKey, "max"))
                            vp->input_ranges.acc.max = atof(scalar);
                    }
                    else if (section == VELOCITY) {
                        if (equals(lastKey, "min"))
                            vp->input_ranges.vel.min = atof(scalar);
                        else if (equals(lastKey, "max"))
                            vp->input_ranges.vel.max = atof(scalar);
                    }
                    else if (section == STEERING) {
                        if (equals(lastKey, "min"))
                            vp->input_ranges.delta.min = atof(scalar);
                        else if (equals(lastKey, "max"))
                            vp->input_ranges.delta.max = atof(scalar);
                    }
                    free(lastKey);
                    lastKey = NULL;
                }
                break;
            }
            case YAML_MAPPING_END_EVENT:
                // When ending a mapping, if we were in a sub-section of input_ranges, revert to INPUT_RANGES.
                if (section == ACCELERATION || section == VELOCITY || section == STEERING) {
                    section = INPUT_RANGES;
                }
                break;
            case YAML_STREAM_END_EVENT:
                done = 1;
                break;
            default:
                break;
        }
        yaml_event_delete(&event);
    }
    
    yaml_parser_delete(&parser);
    fclose(fh);
    
    // Post-process: Compute derived kinematic values.
    vp->kinematic.l_F = vp->kinematic.l * (1.0 - vp->kinematic.w_front);
    vp->kinematic.l_R = vp->kinematic.l * vp->kinematic.w_front;
}
