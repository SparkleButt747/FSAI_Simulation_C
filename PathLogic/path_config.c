#include "path_config.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* 
 * Utility function to generate a random double in [0, 1).
 * Note: srand() should be called (e.g., in main) before using this function.
 */
static double random_double() {
    return (double)rand() / (double)RAND_MAX;
}

/*
 * Calculates the resolution and track length based on the config parameters.
 * The formulas follow the original C# implementation:
 *
 *   t = amplitude * maxFrequency
 *   length = ((0.6387 * t + 43.86) * t + 123.1) * t + 35.9
 *
 *   r = log(length) / log(2) / minCornerRadius
 *   resolution = 4 * length * max(1/minConeSpacing, r/maxConeSpacing)
 */
void calculate_resolution_and_length(PathConfig* config) {
    double t = config->amplitude * config->maxFrequency;
    double length = ((0.6387 * t + 43.86) * t + 123.1) * t + 35.9;
    config->length = length;
    
    double minSep = config->minConeSpacing;
    double maxSep = config->maxConeSpacing;
    /* log base 2: log(length)/log(2) */
    double r = log(length) / log(2.0) / config->minCornerRadius;
    config->resolution = (int)(4 * length * fmax(1 / minSep, r / maxSep));
}

/*
 * Initializes the given PathConfig with default values.
 * These defaults match the C# defaults and then call the resolution/length calculation.
 */
void init_default_path_config(PathConfig* config) {
    /* Ensure that srand() is called once (in main, for example) to seed the random number generator */
    config->seed = random_double();
    config->minCornerRadius = 3.0;
    config->maxFrequency = 6;
    config->amplitude = 1.0 / 3.0;
    config->checkSelfIntersection = 1;  /* true */
    config->startingAmplitude = 0.4;
    config->relativeAccuracy = 0.005;
    config->margin = 0.0;
    config->startingStraightLength = 6.0;
    config->startingStraightDownsample = 2;
    config->minConeSpacing = 3.0 * M_PI / 16.0;
    config->maxConeSpacing = 0.6;
    config->trackWidth = 5.0;
    config->coneSpacingBias = 1.0;
    config->startingConeSpacing = 2.5;
    
    /* Compute derived values */
    calculate_resolution_and_length(config);
}
