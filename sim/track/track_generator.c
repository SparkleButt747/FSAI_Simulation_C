#include "track_generator.h"
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
   Helper Functions for Array Operations on Complex Numbers and Doubles
   --------------------------------------------------------------------------- */

/* Compute Euclidean distance between two complex numbers (treating them as 2D points: real=x, imag=z) */
static double complex_distance(double complex a, double complex b) {
    return cabs(a - b);
}

/* Returns a new array (of length nPoints) where each element is the distance between
   consecutive points in the input array (with the last distance computed between
   the last and first element). Caller must free the result.
*/
static double* calc_distance_to_next(const double complex *points, int nPoints) {
    double *distances = malloc(nPoints * sizeof(double));
    if (!distances) { perror("malloc distances"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints - 1; i++) {
        distances[i] = complex_distance(points[i+1], points[i]);
    }
    distances[nPoints - 1] = complex_distance(points[0], points[nPoints - 1]);
    return distances;
}

/* Rolls (rotates) a double array by 'shift' positions. Returns a new allocated array.
   For example, roll([a,b,c,d], 1) returns [d,a,b,c].
*/
static double* roll_array(const double* arr, int nPoints, int shift) {
    double* rolled = malloc(nPoints * sizeof(double));
    if (!rolled) { perror("malloc rolled"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints; i++) {
        int newIndex = (i - shift + nPoints) % nPoints;
        rolled[newIndex] = arr[i];
    }
    return rolled;
}

/* Multiplies each element of a complex array by a scalar. Returns a new allocated array. */
static double complex* multiply_complex_array_scalar(const double complex* arr, int nPoints, double scalar) {
    double complex* result = malloc(nPoints * sizeof(double complex));
    if (!result) { perror("malloc multiply_complex_array_scalar"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints; i++) {
        result[i] = arr[i] * scalar;
    }
    return result;
}

/* Adds two complex arrays elementwise. Returns a new allocated array.
   Arrays must be of length nPoints.
*/
static double complex* add_complex_arrays(const double complex* a, const double complex* b, int nPoints) {
    double complex* result = malloc(nPoints * sizeof(double complex));
    if (!result) { perror("malloc add_complex_arrays"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints; i++) {
        result[i] = a[i] + b[i];
    }
    return result;
}

/* Subtracts b from a elementwise. Returns a new allocated array.
   Arrays must be of length nPoints.
*/
static double complex* subtract_complex_arrays(const double complex* a, const double complex* b, int nPoints) {
    double complex* result = malloc(nPoints * sizeof(double complex));
    if (!result) { perror("malloc subtract_complex_arrays"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints; i++) {
        result[i] = a[i] - b[i];
    }
    return result;
}

/* Translates (adds a constant to) every element in a double array. Returns a new array.
*/
static double* translate_double_array(const double* arr, int nPoints, double val) {
    double* result = malloc(nPoints * sizeof(double));
    if (!result) { perror("malloc translate_double_array"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints; i++) {
        result[i] = arr[i] + val;
    }
    return result;
}

/* ---------------------------------------------------------------------------
   New Helper: Resample a Boundary Curve
   ---------------------------------------------------------------------------
   Given an array of complex points representing a boundary curve and its length nPoints,
   this function produces a new array with nResample points equally spaced by arc-length.
   Linear interpolation is used between points.
*/
static double complex* resample_boundary(const double complex* points, int nPoints, int nResample) {
    if(nPoints < 2 || nResample < 1) {
        return NULL;
    }
    
    // Compute cumulative arc-length along the boundary.
    double* cum = malloc(nPoints * sizeof(double));
    if (!cum) { perror("malloc cum"); exit(EXIT_FAILURE); }
    cum[0] = 0.0;
    for (int i = 1; i < nPoints; i++) {
         cum[i] = cum[i-1] + cabs(points[i] - points[i-1]);
    }
    double total_length = cum[nPoints - 1];
    
    double complex* resampled = malloc(nResample * sizeof(double complex));
    if (!resampled) { perror("malloc resampled"); exit(EXIT_FAILURE); }
    
    // For each new sample, determine the target distance along the curve.
    for (int j = 0; j < nResample; j++) {
         double target = (nResample == 1) ? 0.0 : (j * total_length / (nResample - 1));
         // Find segment [i, i+1] such that cum[i] <= target <= cum[i+1]
         int i = 0;
         while (i < nPoints - 1 && cum[i+1] < target) {
              i++;
         }
         double segment_length = cum[i+1] - cum[i];
         double fraction = (segment_length > 0) ? (target - cum[i]) / segment_length : 0.0;
         resampled[j] = points[i] + fraction * (points[i+1] - points[i]);
    }
    
    free(cum);
    return resampled;
}

/* ---------------------------------------------------------------------------
   Utility to convert a complex number (used as 2D point: real=x, imag=z) into a Vector3.
   We set y = 0.
*/
static Vector3 complex_to_vector3(double complex z) {
    Vector3 v;
    v.x = creal(z);
    v.y = 0.0;
    v.z = cimag(z);
    return v;
}

/* Compute the rotation (angle in radians) from a complex number interpreted as a vector.
   Uses atan2(imaginary, real).
*/
static double complex_to_angle(double complex z) {
    return atan2(cimag(z), creal(z));
}

/* ---------------------------------------------------------------------------
   Forward Declaration of Cone Placement Function ("Place")
   ---------------------------------------------------------------------------
   This declaration ensures that the function is known before its use in generate_track.
*/
static int place_cones(const double complex *points, const double *radii, int nPoints,
                         int side, double minConeSpacing, double maxConeSpacing,
                         double minCornerRadius, double trackWidth, double coneSpacingBias,
                         double complex **selectedPoints);

/* ---------------------------------------------------------------------------
   Cone Placement Function ("Place")
   ---------------------------------------------------------------------------
   This function selects a subset of points (representing cone positions along a curve)
   using a spacing/density algorithm.
   
   Parameters:
     - points: array of complex points (representing positions).
     - radii: array of doubles (modified corner radii) corresponding to each point.
     - nPoints: number of elements in these arrays.
     - side: an integer (1 for left, -1 for right) affecting density computation.
     - minConeSpacing, maxConeSpacing, minCornerRadius, trackWidth, coneSpacingBias:
             configuration parameters.
     - selectedPoints: output pointer that will be set to a newly allocated array of
             complex numbers (cone positions). Caller is responsible for freeing it.
   
   Returns:
     The number of selected cone points.
*/
static int place_cones(const double complex *points, const double *radii, int nPoints,
                         int side, double minConeSpacing, double maxConeSpacing,
                         double minCornerRadius, double trackWidth, double coneSpacingBias,
                         double complex **selectedPoints) {
    /* Calculate density parameters */
    double minDensity = (1.0 / maxConeSpacing) + 0.1;
    double maxDensity = 1.0 / minConeSpacing;
    double densityRange = maxDensity - minDensity;
    double c1 = densityRange / 2.0 * ((1 - coneSpacingBias) * minCornerRadius - (1 + coneSpacingBias) * trackWidth / 2.0);
    double c2 = densityRange / 2.0 * ((1 + coneSpacingBias) * minCornerRadius - (1 - coneSpacingBias) * trackWidth / 2.0);

    double *distToNext = calc_distance_to_next(points, nPoints);
    double *distToPrev = roll_array(distToNext, nPoints, 1);

    double *coneDensity = malloc(nPoints * sizeof(double));
    if (!coneDensity) { perror("malloc coneDensity"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nPoints; i++) {
        coneDensity[i] = minDensity + (side * c1 / radii[i]) + (c2 / fabs(radii[i]));
    }
    for (int i = 0; i < nPoints; i++) {
        coneDensity[i] *= distToPrev[i];
    }

    double modifiedLength = 0.0;
    for (int i = 0; i < nPoints; i++) {
        modifiedLength += coneDensity[i];
    }
    /* Round modifiedLength to nearest integer */
    int rounded = (int)round(modifiedLength);
    double threshold = (rounded != 0) ? modifiedLength / rounded : 1.0;

    /* Allocate temporary array to store selected cone positions (max possible: nPoints) */
    double complex *temp = malloc(nPoints * sizeof(double complex));
    if (!temp) { perror("malloc temp"); exit(EXIT_FAILURE); }
    int count = 0;
    /* Always include the first point */
    temp[count++] = points[0];

    double current = 0.0;
    for (int i = 1; i < nPoints; i++) {
        current += coneDensity[i];
        if (current >= threshold) {
            current -= threshold;
            temp[count++] = points[i];
        }
    }

    free(distToNext);
    free(distToPrev);
    free(coneDensity);

    /* Resize the array to the actual count */
    *selectedPoints = malloc(count * sizeof(double complex));
    if (!*selectedPoints) { perror("malloc selectedPoints"); exit(EXIT_FAILURE); }
    for (int i = 0; i < count; i++) {
        (*selectedPoints)[i] = temp[i];
    }
    free(temp);
    return count;
}

/* ---------------------------------------------------------------------------
   Main Function: generate_track
   ---------------------------------------------------------------------------
   This function uses the provided path data and configuration to compute:
     - Left cones: computed as (path.points + (trackWidth/2)*path.normals)
     - Right cones: computed as (path.points - (trackWidth/2)*path.normals)
   It then resamples both boundaries to have the same number of points before computing
   the checkpoints as midpoints between corresponding left and right resampled points.
*/
TrackResult generate_track(const PathConfig *config, const PathResult *path) {
    int nPoints = path->nPoints;
    double trackWidth = config->trackWidth;
    double minConeSpacing = config->minConeSpacing;
    double maxConeSpacing = config->maxConeSpacing;
    double coneSpacingBias = config->coneSpacingBias;
    double minCornerRadius = config->minCornerRadius;
    
    /* Compute shifted positions for left and right cones */
    double complex *leftPoints = add_complex_arrays(path->points,
                                  multiply_complex_array_scalar(path->normals, nPoints, trackWidth / 2.0),
                                  nPoints);
    double complex *rightPoints = subtract_complex_arrays(path->points,
                                  multiply_complex_array_scalar(path->normals, nPoints, trackWidth / 2.0),
                                  nPoints);
    
    /* Translate corner radii for left and right sides */
    double *leftRadii = translate_double_array(path->cornerRadii, nPoints, -trackWidth / 2.0);
    double *rightRadii = translate_double_array(path->cornerRadii, nPoints, trackWidth / 2.0);
    
    /* Use the placement function to sample cone positions along the left and right boundaries */
    double complex *leftConesComplex = NULL;
    double complex *rightConesComplex = NULL;
    int nLeftCones = place_cones(leftPoints, leftRadii, nPoints, 1,
                                   minConeSpacing, maxConeSpacing, minCornerRadius, trackWidth, coneSpacingBias,
                                   &leftConesComplex);
    int nRightCones = place_cones(rightPoints, rightRadii, nPoints, -1,
                                   minConeSpacing, maxConeSpacing, minCornerRadius, trackWidth, coneSpacingBias,
                                   &rightConesComplex);
    
    free(leftPoints);
    free(rightPoints);
    free(leftRadii);
    free(rightRadii);
    
    /* Determine the number of samples to use for resampling.
       We choose the minimum of the two counts to ensure both boundaries have equal sample sizes.
    */
    int nResample = (nLeftCones < nRightCones) ? nLeftCones : nRightCones;
    if(nResample < 2) {
        fprintf(stderr, "Not enough cones on one side to compute a racing line.\n");
        exit(EXIT_FAILURE);
    }
    
    /* Resample both boundaries to have nResample uniformly spaced points */
    double complex* leftResampled = resample_boundary(leftConesComplex, nLeftCones, nResample);
    double complex* rightResampled = resample_boundary(rightConesComplex, nRightCones, nResample);
    
    free(leftConesComplex);
    free(rightConesComplex);
    
    /* Convert the resampled boundaries into Transform structs.
       For simplicity, the rotation for each cone is set to 0.
    */
    Transform* leftTransforms = malloc(nResample * sizeof(Transform));
    if (!leftTransforms) { perror("malloc leftTransforms"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nResample; i++) {
        leftTransforms[i].position = complex_to_vector3(leftResampled[i]);
        leftTransforms[i].yaw = 0.0;
    }
    
    Transform* rightTransforms = malloc(nResample * sizeof(Transform));
    if (!rightTransforms) { perror("malloc rightTransforms"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nResample; i++) {
        rightTransforms[i].position = complex_to_vector3(rightResampled[i]);
        rightTransforms[i].yaw = 0.0;
    }
    
    /* Generate checkpoints by computing the midpoint of corresponding left and right resampled points.
       The rotation is computed as the angle from the left point to the right point.
    */
    Transform* checkpoints = malloc(nResample * sizeof(Transform));
    if (!checkpoints) { perror("malloc checkpoints"); exit(EXIT_FAILURE); }
    for (int i = 0; i < nResample; i++) {
        double cx = (creal(leftResampled[i]) + creal(rightResampled[i])) / 2.0;
        double cz = (cimag(leftResampled[i]) + cimag(rightResampled[i])) / 2.0;
        checkpoints[i].position.x = cx;
        checkpoints[i].position.y = 0.0;
        checkpoints[i].position.z = cz;
        double dx = creal(rightResampled[i]) - creal(leftResampled[i]);
        double dz = cimag(rightResampled[i]) - cimag(leftResampled[i]);
        checkpoints[i].yaw = atan2(dz, dx);
    }
    
    free(leftResampled);
    free(rightResampled);
    
    /* Package results */
    TrackResult result;
    result.nLeftCones = nResample;
    result.leftCones = leftTransforms;
    result.nRightCones = nResample;
    result.rightCones = rightTransforms;
    result.nCheckpoints = nResample;
    result.checkpoints = checkpoints;
    
    return result;
}
