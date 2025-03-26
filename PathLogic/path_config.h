#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H

/* 
 * Structure holding configuration parameters for track generation.
 * Note: Boolean values are represented as ints (0 for false, nonzero for true).
 */
typedef struct {
    double seed;
    double minCornerRadius;
    int maxFrequency;
    double amplitude;
    int checkSelfIntersection;  // use 1 for true, 0 for false
    double startingAmplitude;
    double relativeAccuracy;
    double margin;
    double startingStraightLength;
    int startingStraightDownsample;
    double minConeSpacing;
    double maxConeSpacing;
    double trackWidth;
    double coneSpacingBias;
    double startingConeSpacing;
    int resolution;             // computed value
    double length;              // computed track length
} PathConfig;

/* Initializes the PathConfig structure with default values */
void init_default_path_config(PathConfig* config);

/* Calculates the resolution and length based on the current parameters */
void calculate_resolution_and_length(PathConfig* config);

#endif // PATH_CONFIG_H
